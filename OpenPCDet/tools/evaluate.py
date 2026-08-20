#!/usr/bin/env python3
"""Evaluate one tracked object against frame-aligned RTK ground truth."""

import argparse
import csv
import json
from dataclasses import dataclass
from pathlib import Path

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
        "metrics": metrics,
        "outputs": {
            "frame_errors_csv": str(match_csv.resolve()),
            "summary_json": str(summary_json.resolve()),
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


if __name__ == "__main__":
    main()
