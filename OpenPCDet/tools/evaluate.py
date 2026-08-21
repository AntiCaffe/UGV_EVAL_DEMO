#!/usr/bin/env python3
"""Evaluate one tracked object against frame-aligned RTK ground truth."""

import argparse
import csv
import json
import os
from dataclasses import dataclass
from pathlib import Path
import tempfile

import numpy as np

from track_metrics import calculate_tracking_metrics


TRACK_COLUMN_COUNT = 18
RTK_COLUMNS = {
    "frame_id",
    "rtk_is_fresh",
    "p_lidar_x",
    "p_lidar_y",
    "p_lidar_z",
    "v_lidar_x",
    "v_lidar_y",
    "v_lidar_z",
}


@dataclass(frozen=True)
class TrackSample:
    frame_id: str
    track_id: int
    class_id: int
    class_name: str
    score: float
    position: np.ndarray
    velocity: np.ndarray

    @property
    def speed(self):
        return float(np.linalg.norm(self.velocity))


@dataclass(frozen=True)
class RtkSample:
    frame_id: str
    position: np.ndarray
    velocity: np.ndarray

    @property
    def speed(self):
        return float(np.linalg.norm(self.velocity))


def normalize_frame_id(value):
    """Normalize zero-padded frame IDs while retaining non-numeric IDs."""
    stem = Path(str(value).strip()).stem
    try:
        return "%06d" % int(stem)
    except ValueError:
        return stem


def _finite_vector(values):
    vector = np.asarray(values, dtype=np.float64)
    return vector if np.all(np.isfinite(vector)) else None


def load_tracking_results(tracking_dir, class_id=None, min_score=0.0):
    """Load per-frame tracking TXT files, keeping one row per track/frame."""
    tracking_dir = Path(tracking_dir)
    if not tracking_dir.is_dir():
        raise FileNotFoundError(
            "Tracking result directory does not exist: %s" % tracking_dir
        )

    result_files = sorted(tracking_dir.glob("*.txt"))
    if not result_files:
        raise FileNotFoundError(
            "No tracking TXT files found in: %s" % tracking_dir
        )

    samples_by_key = {}
    for result_path in result_files:
        with result_path.open("r", encoding="utf-8") as result_file:
            for line_number, raw_line in enumerate(result_file, start=1):
                line = raw_line.strip()
                if not line or line.startswith("#"):
                    continue
                fields = line.split()
                if len(fields) != TRACK_COLUMN_COUNT:
                    raise ValueError(
                        "%s:%d: expected %d columns, got %d"
                        % (
                            result_path,
                            line_number,
                            TRACK_COLUMN_COUNT,
                            len(fields),
                        )
                    )

                try:
                    frame_id = normalize_frame_id(fields[1])
                    track_id = int(fields[2])
                    row_class_id = int(fields[3])
                    score = float(fields[5])
                    bounds = _finite_vector([float(v) for v in fields[6:12]])
                    velocity = _finite_vector(
                        [float(v) for v in fields[13:16]]
                    )
                except ValueError as error:
                    raise ValueError(
                        "%s:%d: invalid tracking row: %s"
                        % (result_path, line_number, error)
                    ) from error

                if bounds is None or velocity is None:
                    raise ValueError(
                        "%s:%d: tracking position/velocity contains NaN or inf"
                        % (result_path, line_number)
                    )
                if class_id is not None and row_class_id != class_id:
                    continue
                if score < min_score:
                    continue

                position = (bounds[:3] + bounds[3:]) * 0.5
                sample = TrackSample(
                    frame_id=frame_id,
                    track_id=track_id,
                    class_id=row_class_id,
                    class_name=fields[4],
                    score=score,
                    position=position,
                    velocity=velocity,
                )
                key = (frame_id, track_id)
                previous = samples_by_key.get(key)
                if previous is None or sample.score > previous.score:
                    samples_by_key[key] = sample

    samples_by_frame = {}
    for sample in samples_by_key.values():
        samples_by_frame.setdefault(sample.frame_id, []).append(sample)
    for frame_samples in samples_by_frame.values():
        frame_samples.sort(key=lambda sample: sample.track_id)

    if not samples_by_frame:
        filters = "class_id=%s, min_score=%s" % (class_id, min_score)
        raise RuntimeError("No tracking rows remain after filtering: " + filters)
    return samples_by_frame, len(result_files), len(samples_by_key)


def load_rtk_latest(rtk_csv, include_stale=False, antenna_offset=None):
    """Load valid LiDAR-frame RTK position and velocity rows."""
    rtk_csv = Path(rtk_csv)
    if not rtk_csv.is_file():
        raise FileNotFoundError("RTK CSV does not exist: %s" % rtk_csv)

    offset = np.zeros(3, dtype=np.float64)
    if antenna_offset is not None:
        offset = np.asarray(antenna_offset, dtype=np.float64)

    samples = {}
    counts = {"rows": 0, "stale": 0, "nonfinite": 0}
    with rtk_csv.open("r", encoding="utf-8", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        missing = sorted(RTK_COLUMNS.difference(reader.fieldnames or []))
        if missing:
            raise ValueError(
                "RTK CSV is missing columns: %s" % ", ".join(missing)
            )

        for row in reader:
            counts["rows"] += 1
            if not include_stale and int(row["rtk_is_fresh"]) != 1:
                counts["stale"] += 1
                continue
            try:
                position = _finite_vector(
                    [
                        float(row["p_lidar_x"]),
                        float(row["p_lidar_y"]),
                        float(row["p_lidar_z"]),
                    ]
                )
                velocity = _finite_vector(
                    [
                        float(row["v_lidar_x"]),
                        float(row["v_lidar_y"]),
                        float(row["v_lidar_z"]),
                    ]
                )
            except ValueError:
                position = None
                velocity = None
            if position is None or velocity is None:
                counts["nonfinite"] += 1
                continue

            frame_id = normalize_frame_id(row["frame_id"])
            samples[frame_id] = RtkSample(
                frame_id=frame_id,
                position=position - offset,
                velocity=velocity,
            )

    if not samples:
        raise RuntimeError(
            "No valid RTK rows remain. Check calibration columns and "
            "--include-stale-rtk."
        )
    return samples, counts


def select_track_id(
    rtk_by_frame,
    tracks_by_frame,
    association_gate_m=5.0,
    association_dimension="2d",
):
    """Select the ID that is most often the nearest track to RTK GT."""
    dimensions = 2 if association_dimension == "2d" else 3
    wins = {}
    for frame_id, rtk_sample in rtk_by_frame.items():
        candidates = tracks_by_frame.get(frame_id, [])
        if not candidates:
            continue
        ranked = []
        for track_sample in candidates:
            distance = float(
                np.linalg.norm(
                    track_sample.position[:dimensions]
                    - rtk_sample.position[:dimensions]
                )
            )
            ranked.append((distance, track_sample.track_id))
        distance, track_id = min(ranked)
        if distance <= association_gate_m:
            count, distance_sum = wins.get(track_id, (0, 0.0))
            wins[track_id] = (count + 1, distance_sum + distance)

    if not wins:
        raise RuntimeError(
            "No track is within %.3f m of RTK GT. Specify --track-id or "
            "increase --association-gate-m." % association_gate_m
        )

    return min(
        wins,
        key=lambda track_id: (
            -wins[track_id][0],
            wins[track_id][1] / wins[track_id][0],
            track_id,
        ),
    ), wins


def match_track(track_id, rtk_by_frame, tracks_by_frame):
    """Return frame-aligned samples for one persistent track ID."""
    matches = []
    for frame_id, rtk_sample in rtk_by_frame.items():
        selected = next(
            (
                sample
                for sample in tracks_by_frame.get(frame_id, [])
                if sample.track_id == track_id
            ),
            None,
        )
        if selected is not None:
            matches.append((frame_id, selected, rtk_sample))
    return matches


def write_match_csv(output_path, matches):
    fieldnames = [
        "frame_id",
        "track_id",
        "class_id",
        "class_name",
        "score",
        "pred_x",
        "pred_y",
        "pred_z",
        "gt_x",
        "gt_y",
        "gt_z",
        "position_error_x",
        "position_error_y",
        "position_error_z",
        "position_error_3d_m",
        "pred_vx",
        "pred_vy",
        "pred_vz",
        "gt_vx",
        "gt_vy",
        "gt_vz",
        "velocity_error_x",
        "velocity_error_y",
        "velocity_error_z",
        "velocity_error_3d_m_s",
        "pred_speed_m_s",
        "gt_speed_m_s",
        "speed_error_m_s",
    ]
    with output_path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for frame_id, track, rtk in matches:
            position_error = track.position - rtk.position
            velocity_error = track.velocity - rtk.velocity
            writer.writerow(
                {
                    "frame_id": frame_id,
                    "track_id": track.track_id,
                    "class_id": track.class_id,
                    "class_name": track.class_name,
                    "score": "%.6f" % track.score,
                    "pred_x": "%.6f" % track.position[0],
                    "pred_y": "%.6f" % track.position[1],
                    "pred_z": "%.6f" % track.position[2],
                    "gt_x": "%.6f" % rtk.position[0],
                    "gt_y": "%.6f" % rtk.position[1],
                    "gt_z": "%.6f" % rtk.position[2],
                    "position_error_x": "%.6f" % position_error[0],
                    "position_error_y": "%.6f" % position_error[1],
                    "position_error_z": "%.6f" % position_error[2],
                    "position_error_3d_m": "%.6f"
                    % np.linalg.norm(position_error),
                    "pred_vx": "%.6f" % track.velocity[0],
                    "pred_vy": "%.6f" % track.velocity[1],
                    "pred_vz": "%.6f" % track.velocity[2],
                    "gt_vx": "%.6f" % rtk.velocity[0],
                    "gt_vy": "%.6f" % rtk.velocity[1],
                    "gt_vz": "%.6f" % rtk.velocity[2],
                    "velocity_error_x": "%.6f" % velocity_error[0],
                    "velocity_error_y": "%.6f" % velocity_error[1],
                    "velocity_error_z": "%.6f" % velocity_error[2],
                    "velocity_error_3d_m_s": "%.6f"
                    % np.linalg.norm(velocity_error),
                    "pred_speed_m_s": "%.6f" % track.speed,
                    "gt_speed_m_s": "%.6f" % rtk.speed,
                    "speed_error_m_s": "%.6f"
                    % (track.speed - rtk.speed),
                }
            )


def _load_pyplot():
    """Load a non-interactive Matplotlib backend for Docker/SSH."""
    cache_dir = Path(tempfile.gettempdir()) / "ugv_eval_matplotlib"
    cache_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(cache_dir))

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


def _save_figure(plt, figure, output_path, dpi):
    figure.tight_layout()
    figure.savefig(output_path, dpi=dpi, bbox_inches="tight")
    plt.close(figure)


def _match_arrays(matches):
    return (
        np.vstack([track.position for _, track, _ in matches]),
        np.vstack([rtk.position for _, _, rtk in matches]),
        np.vstack([track.velocity for _, track, _ in matches]),
        np.vstack([rtk.velocity for _, _, rtk in matches]),
    )


def _plot_trajectory(
    plt, output_path, predicted_positions, ground_truth_positions, dpi
):
    figure, axis = plt.subplots(figsize=(8, 7))
    axis.plot(
        ground_truth_positions[:, 0],
        ground_truth_positions[:, 1],
        "-o",
        markersize=3,
        linewidth=1.5,
        label="RTK GT",
    )
    axis.plot(
        predicted_positions[:, 0],
        predicted_positions[:, 1],
        "-o",
        markersize=3,
        linewidth=1.5,
        label="Tracking",
    )
    axis.scatter(
        ground_truth_positions[0, 0],
        ground_truth_positions[0, 1],
        marker="s",
        s=55,
        label="Start",
        zorder=3,
    )
    axis.set_title("XY Trajectory: Tracking vs RTK GT")
    axis.set_xlabel("X [m]")
    axis.set_ylabel("Y [m]")
    axis.set_aspect("equal", adjustable="datalim")
    axis.grid(True, alpha=0.3)
    axis.legend()
    _save_figure(plt, figure, output_path, dpi)


def _plot_vector_errors(
    plt,
    output_path,
    predicted,
    ground_truth,
    title,
    unit,
    dpi,
):
    errors = predicted - ground_truth
    error_2d = np.linalg.norm(errors[:, :2], axis=1)
    error_3d = np.linalg.norm(errors, axis=1)
    frame_index = np.arange(len(errors))

    figure, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for axis_index, axis_name in enumerate(("X", "Y", "Z")):
        axes[0].plot(
            frame_index,
            errors[:, axis_index],
            linewidth=1.2,
            label=axis_name,
        )
    axes[0].axhline(0.0, color="black", linewidth=0.8, alpha=0.6)
    axes[0].set_ylabel("Signed error [%s]" % unit)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(ncol=3)

    axes[1].plot(frame_index, error_2d, linewidth=1.4, label="2D")
    axes[1].plot(frame_index, error_3d, linewidth=1.4, label="3D")
    axes[1].set_xlabel("Matched frame index")
    axes[1].set_ylabel("Error magnitude [%s]" % unit)
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    figure.suptitle(title)
    _save_figure(plt, figure, output_path, dpi)


def _plot_speed(
    plt, output_path, predicted_velocities, ground_truth_velocities, dpi
):
    predicted_speed = np.linalg.norm(predicted_velocities, axis=1)
    ground_truth_speed = np.linalg.norm(ground_truth_velocities, axis=1)
    speed_error = predicted_speed - ground_truth_speed
    frame_index = np.arange(len(predicted_speed))

    figure, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(frame_index, ground_truth_speed, label="RTK GT")
    axes[0].plot(frame_index, predicted_speed, label="Tracking")
    axes[0].set_ylabel("Speed [m/s]")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(frame_index, speed_error, color="tab:red")
    axes[1].axhline(0.0, color="black", linewidth=0.8, alpha=0.6)
    axes[1].set_xlabel("Matched frame index")
    axes[1].set_ylabel("Signed error [m/s]")
    axes[1].grid(True, alpha=0.3)
    figure.suptitle("Speed: Tracking vs RTK GT")
    _save_figure(plt, figure, output_path, dpi)


def _metric_values(metrics, names, unit_suffix):
    rmse = [metrics[name]["rmse_" + unit_suffix] for name in names]
    mae = [metrics[name]["mae_" + unit_suffix] for name in names]
    return np.asarray(rmse), np.asarray(mae)


def _plot_metric_summary(plt, output_path, metrics, coverage, dpi):
    figure, axes = plt.subplots(1, 2, figsize=(12, 5))
    width = 0.36

    position_names = ("position_2d", "position_3d")
    position_rmse, position_mae = _metric_values(
        metrics, position_names, "m"
    )
    position_x = np.arange(len(position_names))
    axes[0].bar(position_x - width / 2, position_rmse, width, label="RMSE")
    axes[0].bar(position_x + width / 2, position_mae, width, label="MAE")
    axes[0].set_xticks(position_x, ("Position 2D", "Position 3D"))
    axes[0].set_ylabel("Error [m]")
    axes[0].set_title("Position Metrics")
    axes[0].grid(True, axis="y", alpha=0.3)
    axes[0].legend()

    velocity_names = ("velocity_2d", "velocity_3d", "speed")
    velocity_rmse, velocity_mae = _metric_values(
        metrics, velocity_names, "m_s"
    )
    velocity_x = np.arange(len(velocity_names))
    axes[1].bar(velocity_x - width / 2, velocity_rmse, width, label="RMSE")
    axes[1].bar(velocity_x + width / 2, velocity_mae, width, label="MAE")
    axes[1].set_xticks(
        velocity_x, ("Velocity 2D", "Velocity 3D", "Speed")
    )
    axes[1].set_ylabel("Error [m/s]")
    axes[1].set_title("Velocity and Speed Metrics")
    axes[1].grid(True, axis="y", alpha=0.3)
    axes[1].legend()

    figure.suptitle("Evaluation Summary (Coverage: %.2f%%)" % (coverage * 100))
    _save_figure(plt, figure, output_path, dpi)


def save_evaluation_plots(matches, metrics, coverage, output_dir, dpi=150):
    """Save trajectory, per-frame errors and summary metrics as PNG files."""
    if not matches:
        raise ValueError("matches must contain at least one frame")
    if dpi <= 0:
        raise ValueError("dpi must be greater than zero")

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    plt = _load_pyplot()
    (
        predicted_positions,
        ground_truth_positions,
        predicted_velocities,
        ground_truth_velocities,
    ) = _match_arrays(matches)
    output_paths = {
        "trajectory_xy_png": output_dir / "trajectory_xy.png",
        "position_errors_png": output_dir / "position_errors.png",
        "velocity_errors_png": output_dir / "velocity_errors.png",
        "speed_comparison_png": output_dir / "speed_comparison.png",
        "metrics_summary_png": output_dir / "metrics_summary.png",
    }

    _plot_trajectory(
        plt,
        output_paths["trajectory_xy_png"],
        predicted_positions,
        ground_truth_positions,
        dpi,
    )
    _plot_vector_errors(
        plt,
        output_paths["position_errors_png"],
        predicted_positions,
        ground_truth_positions,
        "Position Error by Matched Frame",
        "m",
        dpi,
    )
    _plot_vector_errors(
        plt,
        output_paths["velocity_errors_png"],
        predicted_velocities,
        ground_truth_velocities,
        "Velocity Error by Matched Frame",
        "m/s",
        dpi,
    )
    _plot_speed(
        plt,
        output_paths["speed_comparison_png"],
        predicted_velocities,
        ground_truth_velocities,
        dpi,
    )
    _plot_metric_summary(
        plt,
        output_paths["metrics_summary_png"],
        metrics,
        coverage,
        dpi,
    )
    return {name: str(path.resolve()) for name, path in output_paths.items()}


def parse_boolean(value):
    """Parse common CLI boolean aliases."""
    normalized = str(value).strip().lower()
    if normalized in {"true", "t", "yes", "y", "1", "on"}:
        return True
    if normalized in {"false", "f", "no", "n", "0", "off"}:
        return False
    raise argparse.ArgumentTypeError(
        "expected one of: true/false, yes/no, 1/0, on/off"
    )


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Compare one persistent OpenPCDet/ByteTrack track with "
            "frame-aligned rtk_latest.csv ground truth."
        )
    )
    parser.add_argument("--tracking-dir", required=True)
    parser.add_argument("--rtk-csv", required=True)
    parser.add_argument("--output-dir", default="evaluation_results")
    parser.add_argument(
        "-v",
        "--visualization",
        type=parse_boolean,
        default=True,
        metavar="{true,false}",
        help="Save evaluation PNG files (boolean aliases are accepted)",
    )
    parser.add_argument(
        "--plot-dpi",
        type=int,
        default=150,
        help="Resolution of saved PNG plots",
    )
    parser.add_argument(
        "--track-id",
        type=int,
        help=(
            "Track ID to evaluate. If omitted, choose the ID most often "
            "nearest to RTK GT."
        ),
    )
    parser.add_argument("--class-id", type=int)
    parser.add_argument("--min-score", type=float, default=0.0)
    parser.add_argument(
        "--association-gate-m",
        type=float,
        default=5.0,
        help="Maximum distance used only for automatic track-ID selection",
    )
    parser.add_argument(
        "--association-dimension",
        choices=("2d", "3d"),
        default="2d",
    )
    parser.add_argument(
        "--include-stale-rtk",
        action="store_true",
        help="Include rows where rtk_is_fresh is not 1",
    )
    parser.add_argument(
        "--antenna-offset-lidar",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        default=(0.0, 0.0, 0.0),
        help=(
            "Antenna position relative to the evaluation target in the "
            "LiDAR frame; subtracted from RTK position"
        ),
    )
    args = parser.parse_args(argv)
    if args.min_score < 0.0:
        parser.error("--min-score must be non-negative")
    if args.association_gate_m <= 0.0:
        parser.error("--association-gate-m must be greater than zero")
    if args.plot_dpi <= 0:
        parser.error("--plot-dpi must be greater than zero")
    return args


def evaluate(args):
    tracks_by_frame, tracking_file_count, tracking_row_count = (
        load_tracking_results(
            args.tracking_dir,
            class_id=args.class_id,
            min_score=args.min_score,
        )
    )
    rtk_by_frame, rtk_counts = load_rtk_latest(
        args.rtk_csv,
        include_stale=args.include_stale_rtk,
        antenna_offset=args.antenna_offset_lidar,
    )

    selection = "explicit"
    association_wins = {}
    selected_track_id = args.track_id
    if selected_track_id is None:
        selection = "automatic_nearest_vote"
        selected_track_id, association_wins = select_track_id(
            rtk_by_frame,
            tracks_by_frame,
            association_gate_m=args.association_gate_m,
            association_dimension=args.association_dimension,
        )

    matches = match_track(selected_track_id, rtk_by_frame, tracks_by_frame)
    if not matches:
        raise RuntimeError("The selected track has no frames overlapping RTK GT")
    metrics = calculate_tracking_metrics(
        predicted_positions=np.vstack(
            [track.position for _, track, _ in matches]
        ),
        ground_truth_positions=np.vstack(
            [rtk.position for _, _, rtk in matches]
        ),
        predicted_velocities=np.vstack(
            [track.velocity for _, track, _ in matches]
        ),
        ground_truth_velocities=np.vstack(
            [rtk.velocity for _, _, rtk in matches]
        ),
    )
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    match_csv = output_dir / "frame_errors.csv"
    summary_json = output_dir / "summary.json"
    write_match_csv(match_csv, matches)

    coverage = len(matches) / len(rtk_by_frame)
    plot_outputs = {}
    if args.visualization:
        plot_outputs = save_evaluation_plots(
            matches=matches,
            metrics=metrics,
            coverage=coverage,
            output_dir=output_dir,
            dpi=args.plot_dpi,
        )
    summary = {
        "inputs": {
            "tracking_dir": str(Path(args.tracking_dir).resolve()),
            "rtk_csv": str(Path(args.rtk_csv).resolve()),
        },
        "selection": {
            "mode": selection,
            "track_id": selected_track_id,
            "class_id_filter": args.class_id,
            "min_score": args.min_score,
            "association_gate_m": args.association_gate_m,
            "association_dimension": args.association_dimension,
            "automatic_wins": {
                str(track_id): {
                    "frames": count,
                    "mean_distance_m": distance_sum / count,
                }
                for track_id, (count, distance_sum) in association_wins.items()
            },
        },
        "counts": {
            "tracking_files": tracking_file_count,
            "tracking_rows_after_filter": tracking_row_count,
            "rtk_rows_total": rtk_counts["rows"],
            "rtk_rows_skipped_stale": rtk_counts["stale"],
            "rtk_rows_skipped_nonfinite": rtk_counts["nonfinite"],
            "rtk_frames_evaluated": len(rtk_by_frame),
            "matched_frames": len(matches),
            "missing_track_frames": len(rtk_by_frame) - len(matches),
            "coverage": coverage,
        },
        "antenna_offset_lidar": list(args.antenna_offset_lidar),
        "visualization": {
            "enabled": args.visualization,
            "dpi": args.plot_dpi,
        },
        "metrics": metrics,
        "outputs": {
            "frame_errors_csv": str(match_csv.resolve()),
            "summary_json": str(summary_json.resolve()),
            **plot_outputs,
        },
        "notes": [
            "RMSE/MAE use only frames where the selected track and valid RTK overlap.",
            "Use coverage and missing_track_frames alongside error metrics.",
            "RTK represents an antenna trajectory unless an antenna offset is supplied.",
        ],
    }
    with summary_json.open("w", encoding="utf-8") as json_file:
        json.dump(summary, json_file, indent=2, sort_keys=True)
        json_file.write("\n")
    return summary


def _metric_line(label, values, unit):
    return "%s RMSE/MAE: %.6f / %.6f %s" % (
        label,
        values["rmse_" + unit],
        values["mae_" + unit],
        unit.replace("_", "/"),
    )


def main(argv=None):
    args = parse_args(argv)
    summary = evaluate(args)
    counts = summary["counts"]
    metrics = summary["metrics"]
    print("Selected track ID: %d" % summary["selection"]["track_id"])
    print(
        "Matched frames: %d/%d (coverage %.2f%%)"
        % (
            counts["matched_frames"],
            counts["rtk_frames_evaluated"],
            counts["coverage"] * 100.0,
        )
    )
    print(_metric_line("Position 2D", metrics["position_2d"], "m"))
    print(_metric_line("Position 3D", metrics["position_3d"], "m"))
    print(_metric_line("Velocity 2D", metrics["velocity_2d"], "m_s"))
    print(_metric_line("Velocity 3D", metrics["velocity_3d"], "m_s"))
    print(_metric_line("Speed", metrics["speed"], "m_s"))
    print("Summary: %s" % summary["outputs"]["summary_json"])
    print("Per-frame errors: %s" % summary["outputs"]["frame_errors_csv"])
    plot_count = sum(key.endswith("_png") for key in summary["outputs"])
    if plot_count:
        print(
            "Plots: %d PNG files in %s"
            % (plot_count, str(Path(args.output_dir).resolve()))
        )


if __name__ == "__main__":
    main()
