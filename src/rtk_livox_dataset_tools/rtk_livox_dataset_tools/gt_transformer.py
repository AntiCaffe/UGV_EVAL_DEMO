import argparse
import csv
import math
import os
from datetime import datetime, timezone

import numpy as np
import yaml

from rtk_livox_dataset_tools.bag_utils import iter_deserialized_messages
from rtk_livox_dataset_tools.geo import enu_backend_name, llh_to_enu, navpvt_to_measurement
from rtk_livox_dataset_tools.rtk_quality import classify_rtk_quality
from rtk_livox_dataset_tools.transforms import rotation_from_yaw


CSV_FIELDS = [
    "stamp_sec",
    "p_lidar_x",
    "p_lidar_y",
    "p_lidar_z",
    "v_lidar_x",
    "v_lidar_y",
    "v_lidar_z",
    "speed_2d",
    "fix_type",
    "flags",
    "h_acc_mm",
    "v_acc_mm",
    "s_acc_mm_s",
    "quality",
]


def _load_calibration(path):
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    required = ["origin_llh", "yaw_enu_lidar_rad", "lidar_position_enu"]
    missing = [key for key in required if key not in data]
    if missing:
        raise RuntimeError("Calibration YAML is missing keys: %s" % ", ".join(missing))
    return data


def _ensure_parent_dir(path):
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)


def transform_navpvt_sample(stamp_sec, fields, calib):
    origin_llh = calib["origin_llh"]
    lidar_position_enu = np.asarray(calib["lidar_position_enu"], dtype=float)
    rotation_lidar_enu = rotation_from_yaw(float(calib["yaw_enu_lidar_rad"])).T

    p_enu = np.asarray(
        llh_to_enu(fields["lat_deg"], fields["lon_deg"], fields["height_m"], origin_llh),
        dtype=float,
    )
    v_enu = np.asarray(
        [fields["vel_e_m_s"], fields["vel_n_m_s"], fields["vel_u_m_s"]],
        dtype=float,
    )

    p_lidar = rotation_lidar_enu.dot(p_enu - lidar_position_enu)
    v_lidar = rotation_lidar_enu.dot(v_enu)
    speed_2d = math.hypot(float(v_lidar[0]), float(v_lidar[1]))
    quality = classify_rtk_quality(
        fields["fix_type"],
        fields["flags"],
        fields["h_acc_mm"],
        fields["v_acc_mm"],
        fields["s_acc_mm_s"],
    )

    return {
        "stamp_sec": "%.9f" % stamp_sec,
        "p_lidar_x": "%.6f" % p_lidar[0],
        "p_lidar_y": "%.6f" % p_lidar[1],
        "p_lidar_z": "%.6f" % p_lidar[2],
        "v_lidar_x": "%.6f" % v_lidar[0],
        "v_lidar_y": "%.6f" % v_lidar[1],
        "v_lidar_z": "%.6f" % v_lidar[2],
        "speed_2d": "%.6f" % speed_2d,
        "fix_type": fields["fix_type"],
        "flags": fields["flags"],
        "h_acc_mm": fields["h_acc_mm"],
        "v_acc_mm": fields["v_acc_mm"],
        "s_acc_mm_s": fields["s_acc_mm_s"],
        "quality": quality,
    }


def write_rtk_gt_csv(rtk_bag, calib_path, output_path, navpvt_topic, storage_id):
    calib = _load_calibration(calib_path)
    rows = []
    quality_counts = {}
    for bag_msg in iter_deserialized_messages(rtk_bag, topics=[navpvt_topic], storage_id=storage_id):
        fields = navpvt_to_measurement(bag_msg.msg)
        row = transform_navpvt_sample(bag_msg.stamp_sec, fields, calib)
        rows.append(row)
        quality_counts[row["quality"]] = quality_counts.get(row["quality"], 0) + 1

    if not rows:
        raise RuntimeError("No NavPVT samples found on topic %s" % navpvt_topic)

    _ensure_parent_dir(output_path)
    with open(output_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    sidecar_path = os.path.splitext(output_path)[0] + ".yaml"
    with open(sidecar_path, "w") as f:
        yaml.safe_dump(
            {
                "rtk_bag": rtk_bag,
                "calib": calib_path,
                "output": output_path,
                "navpvt_topic": navpvt_topic,
                "storage_id": storage_id,
                "enu_backend": enu_backend_name(),
                "sample_count": len(rows),
                "quality_counts": quality_counts,
                "generated_at": datetime.now(timezone.utc).isoformat(),
            },
            f,
            sort_keys=False,
        )

    return len(rows), quality_counts, sidecar_path


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--rtk-bag", required=True)
    parser.add_argument("--calib", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--navpvt-topic", default="/ublox_gps_node/navpvt")
    parser.add_argument("--storage-id", default="sqlite3")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    count, quality_counts, sidecar_path = write_rtk_gt_csv(
        rtk_bag=args.rtk_bag,
        calib_path=args.calib,
        output_path=args.output,
        navpvt_topic=args.navpvt_topic,
        storage_id=args.storage_id,
    )
    print("Wrote RTK GT CSV: %s" % args.output)
    print("Wrote metadata: %s" % sidecar_path)
    print("samples: %d quality_counts: %s" % (count, quality_counts))


if __name__ == "__main__":
    main()
