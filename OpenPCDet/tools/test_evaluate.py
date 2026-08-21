import csv
import importlib.util
import json
from pathlib import Path
import sys

import pytest


MODULE_PATH = Path(__file__).with_name("evaluate.py")
sys.path.insert(0, str(MODULE_PATH.parent))
SPEC = importlib.util.spec_from_file_location("tracking_evaluate", MODULE_PATH)
evaluate_module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(evaluate_module)

from track_metrics import (  # noqa: E402
    calculate_scalar_mae,
    calculate_vector_mae,
)


def _track_row(frame, track_id, center, velocity, score=0.9):
    x, y, z = center
    vx, vy, vz = velocity
    bounds = (x - 0.5, y - 0.5, z - 0.5, x + 0.5, y + 0.5, z + 0.5)
    speed = (vx * vx + vy * vy + vz * vz) ** 0.5
    values = [
        int(frame),
        "%06d.bin" % frame,
        track_id,
        1,
        "Car",
        score,
        *bounds,
        0.0,
        vx,
        vy,
        vz,
        speed,
        0,
    ]
    return "\t".join(str(value) for value in values)


def _write_rtk_csv(path):
    fieldnames = [
        "frame_id",
        "rtk_is_fresh",
        "p_lidar_x",
        "p_lidar_y",
        "p_lidar_z",
        "v_lidar_x",
        "v_lidar_y",
        "v_lidar_z",
    ]
    with path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerow(
            {
                "frame_id": "000000",
                "rtk_is_fresh": 1,
                "p_lidar_x": 0,
                "p_lidar_y": 0,
                "p_lidar_z": 0,
                "v_lidar_x": 1,
                "v_lidar_y": 0,
                "v_lidar_z": 0,
            }
        )
        writer.writerow(
            {
                "frame_id": "000001",
                "rtk_is_fresh": 1,
                "p_lidar_x": 1,
                "p_lidar_y": 0,
                "p_lidar_z": 0,
                "v_lidar_x": 1,
                "v_lidar_y": 0,
                "v_lidar_z": 0,
            }
        )


def test_evaluate_auto_selects_consistent_nearest_track(tmp_path):
    tracking_dir = tmp_path / "tracking"
    tracking_dir.mkdir()
    for frame in (0, 1):
        rows = [
            _track_row(frame, 7, (frame + 1, 0, 0), (2, 0, 0)),
            _track_row(frame, 9, (20, 20, 0), (0, 0, 0)),
        ]
        (tracking_dir / ("%06d.txt" % frame)).write_text(
            "\n".join(rows) + "\n", encoding="utf-8"
        )

    rtk_csv = tmp_path / "rtk_latest.csv"
    output_dir = tmp_path / "evaluation"
    _write_rtk_csv(rtk_csv)
    args = evaluate_module.parse_args(
        [
            "--tracking-dir",
            str(tracking_dir),
            "--rtk-csv",
            str(rtk_csv),
            "--output-dir",
            str(output_dir),
        ]
    )

    summary = evaluate_module.evaluate(args)

    assert summary["selection"]["track_id"] == 7
    assert summary["counts"]["matched_frames"] == 2
    assert summary["counts"]["coverage"] == pytest.approx(1.0)
    assert summary["metrics"]["position_3d"]["rmse_m"] == pytest.approx(1.0)
    assert summary["metrics"]["position_3d"]["mae_m"] == pytest.approx(1.0)
    assert summary["metrics"]["velocity_3d"]["rmse_m_s"] == pytest.approx(1.0)
    assert summary["metrics"]["velocity_3d"]["mae_m_s"] == pytest.approx(1.0)
    assert summary["metrics"]["speed"]["rmse_m_s"] == pytest.approx(1.0)
    assert (output_dir / "frame_errors.csv").is_file()
    expected_plots = {
        "trajectory_xy.png",
        "position_errors.png",
        "velocity_errors.png",
        "speed_comparison.png",
        "metrics_summary.png",
    }
    assert all(
        (output_dir / filename).stat().st_size > 0
        for filename in expected_plots
    )
    saved = json.loads((output_dir / "summary.json").read_text())
    assert saved["selection"]["track_id"] == 7
    assert {
        Path(path).name
        for name, path in saved["outputs"].items()
        if name.endswith("_png")
    } == expected_plots


def test_stale_rtk_is_excluded_by_default(tmp_path):
    rtk_csv = tmp_path / "rtk_latest.csv"
    _write_rtk_csv(rtk_csv)
    rows = list(csv.DictReader(rtk_csv.open(encoding="utf-8")))
    rows[1]["rtk_is_fresh"] = "0"
    with rtk_csv.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    samples, counts = evaluate_module.load_rtk_latest(rtk_csv)

    assert set(samples) == {"000000"}
    assert counts["stale"] == 1


def test_explicit_mae_functions():
    predicted = [[1.0, 0.0, 0.0], [3.0, 0.0, 0.0]]
    ground_truth = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]

    assert calculate_vector_mae(predicted, ground_truth) == pytest.approx(1.5)
    assert calculate_scalar_mae([2.0, 4.0], [1.0, 2.0]) == pytest.approx(1.5)


@pytest.mark.parametrize(
    ("alias", "expected"),
    [("true", True), ("on", True), ("1", True), ("false", False),
     ("off", False), ("0", False)],
)
def test_visualization_boolean_alias(alias, expected):
    args = evaluate_module.parse_args(
        ["--tracking-dir", "tracking", "--rtk-csv", "rtk.csv", "-v", alias]
    )

    assert args.visualization is expected
