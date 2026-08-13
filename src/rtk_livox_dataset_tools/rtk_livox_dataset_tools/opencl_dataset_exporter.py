import argparse
import csv
import os

import numpy as np
import yaml
from sensor_msgs_py import point_cloud2

from rtk_livox_dataset_tools.bag_utils import iter_deserialized_messages


FRAME_DTYPE = np.dtype([("stamp_sec", "<f8"), ("offset", "<u8"), ("count", "<u4")])
POINT_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])
RTK_DTYPE = np.dtype(
    [
        ("stamp_sec", "<f8"),
        ("px", "<f4"),
        ("py", "<f4"),
        ("pz", "<f4"),
        ("vx", "<f4"),
        ("vy", "<f4"),
        ("vz", "<f4"),
        ("speed_2d", "<f4"),
        ("quality", "u1"),
    ]
)

QUALITY_TO_U8 = {"low": 0, "medium": 1, "high": 2}


def _ensure_dir(path):
    os.makedirs(path, exist_ok=True)


def _cloud_field_names(msg):
    names = [field.name for field in msg.fields]
    if "x" not in names or "y" not in names or "z" not in names:
        raise RuntimeError("PointCloud2 must contain x, y, z fields")
    if "intensity" in names:
        return ("x", "y", "z", "intensity"), True
    return ("x", "y", "z"), False


def _cloud_to_points(msg, max_points_per_frame):
    field_names, has_intensity = _cloud_field_names(msg)
    raw = np.asarray(
        list(point_cloud2.read_points(msg, field_names=field_names, skip_nans=True)),
        dtype=np.float32,
    )
    if raw.size == 0:
        return np.zeros(0, dtype=POINT_DTYPE)
    raw = raw.reshape((-1, len(field_names)))
    if max_points_per_frame and raw.shape[0] > max_points_per_frame:
        step = int(np.ceil(float(raw.shape[0]) / float(max_points_per_frame)))
        raw = raw[::step]

    points = np.zeros(raw.shape[0], dtype=POINT_DTYPE)
    points["x"] = raw[:, 0]
    points["y"] = raw[:, 1]
    points["z"] = raw[:, 2]
    if has_intensity:
        points["intensity"] = raw[:, 3]
    return points


def export_pointcloud_bag(bag_uri, output_dir, cloud_topic, storage_id, max_points_per_frame, use_bag_time):
    points_path = os.path.join(output_dir, "points.bin")
    frames_path = os.path.join(output_dir, "frames.bin")
    frames = []
    point_offset = 0
    frame_count = 0
    point_count = 0

    with open(points_path, "wb") as points_file:
        for bag_msg in iter_deserialized_messages(
            bag_uri,
            topics=[cloud_topic],
            storage_id=storage_id,
            use_header_stamp=not use_bag_time,
        ):
            points = _cloud_to_points(bag_msg.msg, max_points_per_frame)
            points.tofile(points_file)
            frames.append((float(bag_msg.stamp_sec), int(point_offset), int(points.shape[0])))
            point_offset += int(points.shape[0])
            frame_count += 1
            point_count += int(points.shape[0])

    if not frames:
        raise RuntimeError("No PointCloud2 messages found on topic %s" % cloud_topic)

    np.asarray(frames, dtype=FRAME_DTYPE).tofile(frames_path)
    return {
        "points_path": points_path,
        "frames_path": frames_path,
        "frame_count": frame_count,
        "point_count": point_count,
    }


def export_rtk_csv(csv_path, output_dir):
    rows = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(
                (
                    float(row["stamp_sec"]),
                    float(row["p_lidar_x"]),
                    float(row["p_lidar_y"]),
                    float(row["p_lidar_z"]),
                    float(row["v_lidar_x"]),
                    float(row["v_lidar_y"]),
                    float(row["v_lidar_z"]),
                    float(row["speed_2d"]),
                    QUALITY_TO_U8.get(row["quality"], 0),
                )
            )
    if not rows:
        raise RuntimeError("No RTK rows found in %s" % csv_path)
    rtk_path = os.path.join(output_dir, "rtk_gt.bin")
    np.asarray(rows, dtype=RTK_DTYPE).tofile(rtk_path)
    return {"rtk_path": rtk_path, "rtk_count": len(rows)}


def write_metadata(output_dir, args, point_info, rtk_info):
    metadata = {
        "format": "rtk_livox_opencl_dataset_v1",
        "bag": args.bag,
        "cloud_topic": args.cloud_topic,
        "cloud_time_source": args.cloud_time_source,
        "rtk_gt_csv": args.rtk_gt_csv,
        "storage_id": args.storage_id,
        "max_points_per_frame": args.max_points_per_frame,
        "time_policy": {
            "evaluation_clock": "livox_or_tracker_timestamp",
            "rtk_policy": "interpolate RTK GT to each LiDAR/tracker timestamp; do not drop pointcloud frames to RTK Hz",
        },
        "files": {
            "points": "points.bin",
            "frames": "frames.bin",
            "rtk_gt": "rtk_gt.bin",
        },
        "dtypes": {
            "points": POINT_DTYPE.descr,
            "frames": FRAME_DTYPE.descr,
            "rtk_gt": RTK_DTYPE.descr,
        },
        "counts": {
            "frames": point_info["frame_count"],
            "points": point_info["point_count"],
            "rtk_gt": rtk_info["rtk_count"],
        },
    }
    path = os.path.join(output_dir, "metadata.yaml")
    with open(path, "w") as f:
        yaml.safe_dump(metadata, f, sort_keys=False)
    return path


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--rtk-gt-csv", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--cloud-topic", default="/livox/lidar")
    parser.add_argument("--storage-id", default="sqlite3")
    parser.add_argument("--max-points-per-frame", type=int, default=30000)
    parser.add_argument(
        "--cloud-time-source",
        choices=["bag", "header"],
        default="bag",
        help="Use rosbag record time for LiDAR frames by default because some Livox headers use sensor-relative time.",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    _ensure_dir(args.output_dir)
    point_info = export_pointcloud_bag(
        args.bag,
        args.output_dir,
        args.cloud_topic,
        args.storage_id,
        args.max_points_per_frame,
        args.cloud_time_source == "bag",
    )
    rtk_info = export_rtk_csv(args.rtk_gt_csv, args.output_dir)
    metadata_path = write_metadata(args.output_dir, args, point_info, rtk_info)
    print("Wrote OpenCL dataset directory: %s" % args.output_dir)
    print("frames: %d points: %d rtk: %d" % (point_info["frame_count"], point_info["point_count"], rtk_info["rtk_count"]))
    print("metadata: %s" % metadata_path)


if __name__ == "__main__":
    main()
