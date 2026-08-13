import argparse
import math
import os
from dataclasses import dataclass
from datetime import datetime, timezone

import numpy as np
import yaml

from rtk_livox_dataset_tools.geo import enu_backend_name, llh_to_enu, navpvt_to_measurement
from rtk_livox_dataset_tools.rtk_quality import HIGH_QUALITY, LOW_QUALITY, MEDIUM_QUALITY, classify_rtk_quality
from rtk_livox_dataset_tools.transforms import (
    angle_diff_rad,
    circular_std,
    rotation_from_yaw,
    yaw_from_vector,
)

CONVENTION = (
    "yaw=atan2(vN,vE); up-axis; CCW+; East=0. R_enu_lidar maps LiDAR->ENU; "
    "transpose for ENU->LiDAR."
)


@dataclass
class NavSample:
    t_rel: float
    lat_deg: float
    lon_deg: float
    height_m: float
    vel_e_m_s: float
    vel_n_m_s: float
    vel_u_m_s: float
    h_acc_mm: int
    v_acc_mm: int
    s_acc_mm_s: int
    fix_type: int
    flags: int
    quality: str


def parse_window(text):
    try:
        start, end = [float(part) for part in text.split(":", 1)]
    except (AttributeError, ValueError) as exc:
        raise argparse.ArgumentTypeError("window must be START:END seconds") from exc
    if end < start:
        raise argparse.ArgumentTypeError("window end must be >= start")
    return [start, end]


def parse_origin_llh(text):
    try:
        values = [float(part) for part in text.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError("--origin-llh must be lat,lon,height") from exc
    if len(values) != 3:
        raise argparse.ArgumentTypeError("--origin-llh must be lat,lon,height")
    return values


def load_antenna_offset(path):
    if not path or not os.path.exists(path):
        return np.zeros(3, dtype=float), "Antenna offset file not found; using [0, 0, 0]."
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    value = data.get("p_antenna_in_lidar", data)
    if isinstance(value, dict):
        try:
            return np.array([float(value["x"]), float(value["y"]), float(value["z"])], dtype=float), None
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("p_antenna_in_lidar must contain x, y, z") from exc
    if isinstance(value, (list, tuple)) and len(value) == 3:
        return np.array([float(v) for v in value], dtype=float), None
    raise ValueError("Antenna offset YAML must contain p_antenna_in_lidar: [x, y, z]")


def read_navpvt_samples(bag_uri, navpvt_topic, storage_id):
    from rtk_livox_dataset_tools.bag_utils import iter_deserialized_messages

    raw = []
    for bag_msg in iter_deserialized_messages(bag_uri, topics=[navpvt_topic], storage_id=storage_id):
        fields = navpvt_to_measurement(bag_msg.msg)
        quality = classify_rtk_quality(
            fields["fix_type"],
            fields["flags"],
            fields["h_acc_mm"],
            fields["v_acc_mm"],
            fields["s_acc_mm_s"],
        )
        raw.append((bag_msg.stamp_sec, fields, quality))
    raw.sort(key=lambda item: item[0])
    if not raw:
        return []
    first_stamp = raw[0][0]
    return [
        NavSample(
            t_rel=stamp - first_stamp,
            quality=quality,
            **fields,
        )
        for stamp, fields, quality in raw
    ]


def _window_samples(samples, window):
    return [sample for sample in samples if window[0] <= sample.t_rel <= window[1]]


def _speed_2d(sample):
    return math.hypot(sample.vel_e_m_s, sample.vel_n_m_s)


def _weight_from_acc_mm(value, eps_m):
    acc_m = max(float(value) * 1.0e-3, eps_m)
    return 1.0 / (acc_m * acc_m)


def _weighted_mean_velocity(samples):
    if not samples:
        raise ValueError("No moving samples remain after speed filtering")
    weights = np.array([_weight_from_acc_mm(sample.s_acc_mm_s, 1.0e-3) for sample in samples])
    vel_e = np.array([sample.vel_e_m_s for sample in samples])
    vel_n = np.array([sample.vel_n_m_s for sample in samples])
    return float(np.average(vel_e, weights=weights)), float(np.average(vel_n, weights=weights)), weights


def _weighted_mean_position(enu_points, samples):
    weights = np.array([_weight_from_acc_mm(sample.h_acc_mm, 1.0e-3) for sample in samples])
    return np.average(enu_points, axis=0, weights=weights)


def _quality_counts(samples):
    return {
        "high": sum(1 for sample in samples if sample.quality == HIGH_QUALITY),
        "medium": sum(1 for sample in samples if sample.quality == MEDIUM_QUALITY),
        "excluded": sum(1 for sample in samples if sample.quality == LOW_QUALITY),
    }


def _stats(values):
    if not values:
        return {"min": None, "mean": None, "max": None}
    arr = np.asarray(values, dtype=float)
    return {"min": float(np.min(arr)), "mean": float(np.mean(arr)), "max": float(np.max(arr))}


def _rtk_quality_summary(samples):
    counts = _quality_counts(samples)
    return {
        "n_high": counts["high"],
        "n_medium": counts["medium"],
        "n_excluded": counts["excluded"],
        "hAcc_mm": _stats([sample.h_acc_mm for sample in samples]),
        "sAcc_mm_s": _stats([sample.s_acc_mm_s for sample in samples]),
    }


def _select_quality_for_windows(samples, windows, min_samples, warnings):
    high = [sample for sample in samples if sample.quality == HIGH_QUALITY]
    use_medium = False
    for name, window in windows.items():
        if window is None:
            continue
        count = len(_window_samples(high, window))
        if count < min_samples:
            warnings.append(
                "%s window has %d high-quality samples; including medium-quality RTK samples."
                % (name, count)
            )
            use_medium = True
    if use_medium:
        return [sample for sample in samples if sample.quality in (HIGH_QUALITY, MEDIUM_QUALITY)], "high+medium"
    return high, "high"


def _select_quality_for_windows_allow_low(samples, warnings):
    warnings.append("Low-quality RTK samples are included by request; calibration accuracy is not guaranteed.")
    return list(samples), "all"


def _moving_samples(samples, speed_min):
    return [sample for sample in samples if _speed_2d(sample) >= speed_min]


def _stationary_samples(samples, speed_max):
    return [sample for sample in samples if _speed_2d(sample) <= speed_max]


def _enu_points(samples, origin_llh):
    return np.array(
        [llh_to_enu(sample.lat_deg, sample.lon_deg, sample.height_m, origin_llh) for sample in samples],
        dtype=float,
    )


def _pca_yaw_diff_deg(samples, enu_points, yaw):
    if len(samples) < 2:
        return None
    xy = enu_points[:, :2]
    centered = xy - np.mean(xy, axis=0)
    if np.linalg.norm(centered) <= 0.0:
        return None
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    axis = vh[0]
    yaw_axis = np.array([math.cos(yaw), math.sin(yaw)])
    if np.dot(axis, yaw_axis) < 0.0:
        axis = -axis
    pca_yaw = yaw_from_vector(float(axis[0]), float(axis[1]))
    return abs(math.degrees(angle_diff_rad(pca_yaw, yaw)))


def compute_calibration(
    samples,
    run_id,
    navpvt_topic,
    forward_window,
    stationary_window,
    backward_window=None,
    antenna_offset=None,
    origin_llh=None,
    forward_speed_min=0.3,
    stationary_speed_max=0.05,
    min_samples=20,
    backward_angle_tolerance_deg=15.0,
    allow_low_quality=False,
):
    warnings = []
    antenna_offset = np.zeros(3, dtype=float) if antenna_offset is None else np.asarray(antenna_offset, dtype=float)
    valid_qualities = (HIGH_QUALITY, MEDIUM_QUALITY, LOW_QUALITY) if allow_low_quality else (HIGH_QUALITY, MEDIUM_QUALITY)
    valid = [sample for sample in samples if sample.quality in valid_qualities]
    if not valid:
        if allow_low_quality:
            raise RuntimeError("No RTK samples are available")
        raise RuntimeError("No high or medium quality RTK samples are available")
    if origin_llh is None:
        first = valid[0]
        origin_llh = [first.lat_deg, first.lon_deg, first.height_m]
    else:
        origin_llh = [float(v) for v in origin_llh]

    windows = {"forward": forward_window, "backward": backward_window, "stationary": stationary_window}
    if allow_low_quality:
        usable, quality_used = _select_quality_for_windows_allow_low(samples, warnings)
    else:
        usable, quality_used = _select_quality_for_windows(samples, windows, min_samples, warnings)
    forward_all = _window_samples(usable, forward_window)
    backward_all = _window_samples(usable, backward_window) if backward_window else []
    stationary_all = _window_samples(usable, stationary_window)

    forward = _moving_samples(forward_all, forward_speed_min)
    backward = _moving_samples(backward_all, forward_speed_min)
    stationary = _stationary_samples(stationary_all, stationary_speed_max)
    if len(forward) < min_samples:
        warnings.append("Forward moving samples below minimum: %d < %d." % (len(forward), min_samples))
    if backward_window and len(backward) < min_samples:
        warnings.append("Backward moving samples below minimum: %d < %d." % (len(backward), min_samples))
    if len(stationary) < min_samples:
        warnings.append("Stationary samples below minimum: %d < %d." % (len(stationary), min_samples))
    if not forward:
        raise RuntimeError("Forward window has no samples above speed threshold")
    if not stationary:
        raise RuntimeError("Stationary window has no samples below speed threshold")

    fwd_e, fwd_n, fwd_weights = _weighted_mean_velocity(forward)
    yaw_forward = yaw_from_vector(fwd_e, fwd_n)
    combined_e = [sample.vel_e_m_s for sample in forward]
    combined_n = [sample.vel_n_m_s for sample in forward]
    combined_w = [_weight_from_acc_mm(sample.s_acc_mm_s, 1.0e-3) for sample in forward]
    backward_yaw = None
    fwd_bwd_angle_diff_deg = None
    fwd_vs_backward_plus_180_diff_deg = None
    if backward:
        bwd_e, bwd_n, bwd_weights = _weighted_mean_velocity(backward)
        backward_yaw = yaw_from_vector(bwd_e, bwd_n)
        fwd_bwd_angle_diff_deg = abs(math.degrees(angle_diff_rad(backward_yaw, yaw_forward)))
        fwd_vs_backward_plus_180_diff_deg = abs(math.degrees(angle_diff_rad(backward_yaw + math.pi, yaw_forward)))
        if fwd_vs_backward_plus_180_diff_deg > backward_angle_tolerance_deg:
            warnings.append(
                "Forward vs backward+180 heading difference %.2f deg exceeds %.2f deg."
                % (fwd_vs_backward_plus_180_diff_deg, backward_angle_tolerance_deg)
            )
        combined_e.extend([-sample.vel_e_m_s for sample in backward])
        combined_n.extend([-sample.vel_n_m_s for sample in backward])
        combined_w.extend(list(bwd_weights))

    mean_e = float(np.average(np.asarray(combined_e), weights=np.asarray(combined_w)))
    mean_n = float(np.average(np.asarray(combined_n), weights=np.asarray(combined_w)))
    yaw = yaw_from_vector(mean_e, mean_n)
    per_sample_yaws = [yaw_from_vector(e, n) for e, n in zip(combined_e, combined_n)]
    yaw_std_deg = math.degrees(circular_std(per_sample_yaws, combined_w))

    r_enu_lidar = rotation_from_yaw(yaw)
    stationary_enu = _enu_points(stationary, origin_llh)
    p_antenna_enu = _weighted_mean_position(stationary_enu, stationary)
    p_lidar_enu = p_antenna_enu - r_enu_lidar.dot(antenna_offset)

    pca_samples = forward + backward
    pca_points = _enu_points(pca_samples, origin_llh)
    yaw_pca_diff_deg = _pca_yaw_diff_deg(pca_samples, pca_points, yaw) if len(pca_samples) >= 2 else None
    if yaw_pca_diff_deg is not None and yaw_pca_diff_deg > 15.0:
        warnings.append("Velocity yaw and PCA yaw differ by %.2f deg." % yaw_pca_diff_deg)

    stationary_std = np.std(stationary_enu, axis=0)
    return {
        "run_id": run_id,
        "navpvt_topic": navpvt_topic,
        "convention": CONVENTION,
        "enu_backend": enu_backend_name(),
        "origin_llh": [float(v) for v in origin_llh],
        "yaw_enu_lidar_rad": float(yaw),
        "yaw_enu_lidar_deg": float(math.degrees(yaw)),
        "lidar_position_enu": [float(v) for v in p_lidar_enu],
        "antenna_stationary_mean_enu": [float(v) for v in p_antenna_enu],
        "p_antenna_in_lidar": [float(v) for v in antenna_offset],
        "windows_rel_sec": {
            "forward": [float(v) for v in forward_window],
            "backward": [float(v) for v in backward_window] if backward_window else None,
            "stationary": [float(v) for v in stationary_window],
        },
        "qc": {
            "forward_yaw_deg": float(math.degrees(yaw_forward)),
            "backward_yaw_deg": float(math.degrees(backward_yaw)) if backward_yaw is not None else None,
            "fwd_bwd_angle_diff_deg": float(fwd_bwd_angle_diff_deg) if fwd_bwd_angle_diff_deg is not None else None,
            "fwd_vs_backward_plus_180_diff_deg": (
                float(fwd_vs_backward_plus_180_diff_deg)
                if fwd_vs_backward_plus_180_diff_deg is not None
                else None
            ),
            "yaw_std_deg": float(yaw_std_deg),
            "yaw_pca_diff_deg": float(yaw_pca_diff_deg) if yaw_pca_diff_deg is not None else None,
            "stationary_pos_std_m": [float(v) for v in stationary_std],
            "n_samples_forward": len(forward),
            "n_samples_backward": len(backward),
            "n_samples_stationary": len(stationary),
            "quality_used": quality_used,
            "warnings": warnings,
        },
        "rtk_quality_summary": _rtk_quality_summary(samples),
        "generated_at": datetime.now(timezone.utc).isoformat(),
    }


def default_output_path(run_id):
    return os.path.join("calibration", "%s_lidar_rtk_alignment.yaml" % run_id)


def write_yaml(path, data):
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--forward-window", type=parse_window, required=True)
    parser.add_argument("--backward-window", type=parse_window)
    parser.add_argument("--stationary-window", type=parse_window, required=True)
    parser.add_argument("--antenna-offset", default="config/antenna_in_lidar.yaml")
    parser.add_argument("--output")
    parser.add_argument("--navpvt-topic", default="/ublox_gps_node/navpvt")
    parser.add_argument("--storage-id", default="sqlite3")
    parser.add_argument("--forward-speed-min", type=float, default=0.3)
    parser.add_argument("--stationary-speed-max", type=float, default=0.05)
    parser.add_argument("--min-samples", type=int, default=20)
    parser.add_argument("--origin-llh", type=parse_origin_llh)
    parser.add_argument("--allow-low-quality", action="store_true")
    return parser.parse_args(argv)


def calibrate(args):
    samples = read_navpvt_samples(args.bag, args.navpvt_topic, args.storage_id)
    if not samples:
        raise RuntimeError("No NavPVT samples found on topic %s" % args.navpvt_topic)
    antenna_offset, warning = load_antenna_offset(args.antenna_offset)
    result = compute_calibration(
        samples=samples,
        run_id=args.run_id,
        navpvt_topic=args.navpvt_topic,
        forward_window=args.forward_window,
        backward_window=args.backward_window,
        stationary_window=args.stationary_window,
        antenna_offset=antenna_offset,
        origin_llh=args.origin_llh,
        forward_speed_min=args.forward_speed_min,
        stationary_speed_max=args.stationary_speed_max,
        min_samples=args.min_samples,
        allow_low_quality=args.allow_low_quality,
    )
    if warning:
        result["qc"]["warnings"].insert(0, warning)
    return result


def main(argv=None):
    args = parse_args(argv)
    if args.output is None:
        args.output = default_output_path(args.run_id)
    result = calibrate(args)
    write_yaml(args.output, result)
    qc = result["qc"]
    print("Wrote LiDAR pose calibration: %s" % args.output)
    print("yaw_enu_lidar_deg: %.3f" % result["yaw_enu_lidar_deg"])
    print(
        "lidar_position_enu: [%.3f, %.3f, %.3f]"
        % tuple(result["lidar_position_enu"])
    )
    print(
        "qc: fwd_bwd_angle_diff=%s fwd_vs_bwd_plus_180_diff=%s yaw_std=%.3f quality_used=%s"
        % (
            "%.3f" % qc["fwd_bwd_angle_diff_deg"] if qc["fwd_bwd_angle_diff_deg"] is not None else "n/a",
            (
                "%.3f" % qc["fwd_vs_backward_plus_180_diff_deg"]
                if qc.get("fwd_vs_backward_plus_180_diff_deg") is not None
                else "n/a"
            ),
            qc["yaw_std_deg"],
            qc["quality_used"],
        )
    )
    if qc["warnings"]:
        print("warnings:")
        for warning in qc["warnings"]:
            print("  - %s" % warning)
