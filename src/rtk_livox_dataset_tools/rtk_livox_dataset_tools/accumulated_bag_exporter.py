"""Export causal accumulated Livox frames with latest RTK samples."""

import argparse
from bisect import bisect_right
from collections import deque
import csv
import heapq
import math
import os

import numpy as np
import yaml

from rtk_livox_dataset_tools.geo import llh_to_enu
from rtk_livox_dataset_tools.transforms import rotation_from_yaw


POINT_FIELD_DTYPES = {
    1: "i1",
    2: "u1",
    3: "i2",
    4: "u2",
    5: "i4",
    6: "u4",
    7: "f4",
    8: "f8",
}

FRAME_FIELDS = [
    "frame_id",
    "file",
    "stamp_sec",
    "relative_time_sec",
    "window_start_sec",
    "window_end_sec",
    "lidar_packet_count",
    "raw_point_count",
    "output_point_count",
]

RTK_FIELDS = [
    "frame_id",
    "stamp_sec",
    "fix_stamp_sec",
    "fix_age_sec",
    "velocity_stamp_sec",
    "velocity_age_sec",
    "rtk_is_fresh",
    "latitude_deg",
    "longitude_deg",
    "altitude_m",
    "navsat_status",
    "position_covariance_type",
    "position_covariance_xx",
    "position_covariance_yy",
    "position_covariance_zz",
    "velocity_x_raw",
    "velocity_y_raw",
    "velocity_z_raw",
    "velocity_covariance_xx",
    "velocity_covariance_yy",
    "velocity_covariance_zz",
    "p_lidar_x",
    "p_lidar_y",
    "p_lidar_z",
    "v_lidar_x",
    "v_lidar_y",
    "v_lidar_z",
    "speed_2d_m_s",
    "heading_lidar_rad",
]


def _pointcloud_dtype(msg):
    """Build a structured NumPy dtype from PointCloud2 field metadata."""
    byte_order = ">" if msg.is_bigendian else "<"
    names, formats, offsets = [], [], []
    for field in msg.fields:
        if field.datatype not in POINT_FIELD_DTYPES:
            continue
        field_dtype = byte_order + POINT_FIELD_DTYPES[field.datatype]
        if field.count > 1:
            field_dtype = np.dtype((field_dtype, (field.count,)))
        names.append(field.name)
        formats.append(field_dtype)
        offsets.append(field.offset)
    return np.dtype(
        {
            "names": names,
            "formats": formats,
            "offsets": offsets,
            "itemsize": msg.point_step,
        }
    )


def pointcloud2_to_xyzi(msg):
    """Decode PointCloud2 into OpenPCDet-compatible float32 XYZI rows."""
    dtype = _pointcloud_dtype(msg)
    required = {"x", "y", "z"}
    if not required.issubset(dtype.names or ()):
        raise RuntimeError("PointCloud2 must contain x, y and z fields")
    cloud = np.ndarray(
        shape=(msg.height, msg.width),
        dtype=dtype,
        buffer=memoryview(msg.data),
        strides=(msg.row_step, msg.point_step),
    ).reshape(-1)
    points = np.empty((cloud.size, 4), dtype=np.float32)
    points[:, 0] = cloud["x"]
    points[:, 1] = cloud["y"]
    points[:, 2] = cloud["z"]
    if "intensity" in (dtype.names or ()):
        points[:, 3] = cloud["intensity"]
    else:
        points[:, 3] = 0.0
    return points[np.isfinite(points[:, :3]).all(axis=1)]


def reduce_points(points, voxel_size=0.0, max_points=0):
    """Apply deterministic voxel selection and optional uniform limiting."""
    if voxel_size > 0.0 and len(points):
        voxel_keys = np.floor(points[:, :3] / voxel_size).astype(np.int64)
        _, selected = np.unique(voxel_keys, axis=0, return_index=True)
        points = points[np.sort(selected)]
    if max_points > 0 and len(points) > max_points:
        selected = np.linspace(
            0, len(points) - 1, num=max_points, dtype=np.int64
        )
        points = points[selected]
    return np.ascontiguousarray(points, dtype=np.float32)


class CausalCloudAccumulator:
    """Create end-aligned windows containing only packets at or before t."""

    def __init__(self, accumulation_sec, output_rate_hz):
        if accumulation_sec <= 0.0:
            raise ValueError("accumulation_sec must be positive")
        if output_rate_hz <= 0.0:
            raise ValueError("output_rate_hz must be positive")
        self.accumulation_sec = float(accumulation_sec)
        self.period_sec = 1.0 / float(output_rate_hz)
        self.clouds = deque()
        self.next_stamp = None
        self.last_cloud_stamp = None

    def add_cloud(self, stamp_sec, points):
        stamp_sec = float(stamp_sec)
        if (
            self.last_cloud_stamp is not None
            and stamp_sec < self.last_cloud_stamp
        ):
            raise RuntimeError("Bag timestamps must be monotonic")
        self.clouds.append((stamp_sec, points))
        self.last_cloud_stamp = stamp_sec
        if self.next_stamp is None:
            self.next_stamp = stamp_sec + self.accumulation_sec

    def pop_due(self, before_stamp_sec):
        """Yield windows ending strictly before the next bag event."""
        while (
            self.next_stamp is not None
            and self.next_stamp < float(before_stamp_sec)
        ):
            yield self._pop_one()

    def pop_through(self, final_stamp_sec):
        """Yield remaining windows through the last LiDAR packet timestamp."""
        while (
            self.next_stamp is not None
            and self.next_stamp <= float(final_stamp_sec)
        ):
            yield self._pop_one()

    def _pop_one(self):
        stamp_sec = self.next_stamp
        window_start = stamp_sec - self.accumulation_sec
        while self.clouds and self.clouds[0][0] <= window_start:
            self.clouds.popleft()
        packets = [
            points
            for packet_stamp, points in self.clouds
            if packet_stamp <= stamp_sec
        ]
        self.next_stamp += self.period_sec
        return stamp_sec, window_start, packets


def _load_calibration(path):
    if not path:
        return None
    with open(path, "r") as calibration_file:
        calibration = yaml.safe_load(calibration_file) or {}
    required = ["origin_llh", "yaw_enu_lidar_rad", "lidar_position_enu"]
    missing = [key for key in required if key not in calibration]
    if missing:
        raise RuntimeError(
            "Calibration YAML is missing keys: %s" % ", ".join(missing)
        )
    return calibration


def _header_stamp_sec(message):
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _estimate_timing(args, message_iterator):
    """Estimate header alignment and a safe rosbag write reorder margin."""
    if args.time_source == "bag":
        return {
            "cloud_header_to_epoch_sec": 0.0,
            "reorder_slack_sec": 0.0,
            "cloud_offset_mad_sec": 0.0,
        }
    cloud_offsets = []
    write_delays = []
    topics = [args.cloud_topic, args.fix_topic, args.velocity_topic]
    for bag_message in message_iterator(
        args.bag,
        topics=topics,
        storage_id=args.storage_id,
        use_header_stamp=False,
    ):
        header_stamp = _header_stamp_sec(bag_message.msg)
        if bag_message.topic == args.cloud_topic:
            cloud_offsets.append(bag_message.stamp_sec - header_stamp)
        else:
            write_delays.append(bag_message.stamp_sec - header_stamp)
    if not cloud_offsets:
        raise RuntimeError(
            "No PointCloud2 messages found on %s" % args.cloud_topic
        )
    cloud_offset = float(np.median(cloud_offsets))
    cloud_mad = float(
        np.median(np.abs(np.asarray(cloud_offsets) - cloud_offset))
    )
    aligned_cloud_delays = [offset - cloud_offset for offset in cloud_offsets]
    max_write_delay = max(aligned_cloud_delays + write_delays + [0.0])
    return {
        "cloud_header_to_epoch_sec": cloud_offset,
        "reorder_slack_sec": max(0.0, float(max_write_delay)) + 0.01,
        "cloud_offset_mad_sec": cloud_mad,
    }


def _latest_at_or_before(stamps, samples, frame_stamp):
    index = bisect_right(stamps, frame_stamp) - 1
    if index < 0:
        return None
    return stamps[index], samples[index]


def _append_time_ordered(stamps, samples, stamp, sample):
    """Keep header samples ordered despite delayed rosbag writes."""
    index = bisect_right(stamps, stamp)
    stamps.insert(index, stamp)
    samples.insert(index, sample)


def _rtk_row(
    frame_id,
    frame_stamp,
    fix_sample,
    velocity_sample,
    calibration,
    max_rtk_age_sec,
):
    fix_stamp, fix = fix_sample
    velocity_stamp, velocity = velocity_sample
    linear = velocity.twist.twist.linear
    position_covariance = fix.position_covariance
    velocity_covariance = velocity.twist.covariance
    raw_velocity = np.array([linear.x, linear.y, linear.z], dtype=float)
    lidar_position = np.full(3, np.nan)
    lidar_velocity = np.full(3, np.nan)
    speed_2d = float(math.hypot(raw_velocity[0], raw_velocity[1]))
    heading = float("nan")

    if calibration is not None:
        position_enu = np.asarray(
            llh_to_enu(
                fix.latitude,
                fix.longitude,
                fix.altitude,
                calibration["origin_llh"],
            ),
            dtype=float,
        )
        rotation_lidar_enu = rotation_from_yaw(
            float(calibration["yaw_enu_lidar_rad"])
        ).T
        lidar_position = rotation_lidar_enu.dot(
            position_enu
            - np.asarray(calibration["lidar_position_enu"], dtype=float)
        )
        # ublox_gps/fix_velocity follows the ENU convention: x east,
        # y north, z up.  Preserve raw values above for auditability.
        lidar_velocity = rotation_lidar_enu.dot(raw_velocity)
        speed_2d = float(math.hypot(lidar_velocity[0], lidar_velocity[1]))
        if speed_2d > 1.0e-3:
            heading = float(math.atan2(lidar_velocity[1], lidar_velocity[0]))

    return {
        "frame_id": "%06d" % frame_id,
        "stamp_sec": "%.9f" % frame_stamp,
        "fix_stamp_sec": "%.9f" % fix_stamp,
        "fix_age_sec": "%.6f" % (frame_stamp - fix_stamp),
        "velocity_stamp_sec": "%.9f" % velocity_stamp,
        "velocity_age_sec": "%.6f" % (frame_stamp - velocity_stamp),
        "rtk_is_fresh": int(
            frame_stamp - fix_stamp <= max_rtk_age_sec
            and frame_stamp - velocity_stamp <= max_rtk_age_sec
        ),
        "latitude_deg": "%.10f" % fix.latitude,
        "longitude_deg": "%.10f" % fix.longitude,
        "altitude_m": "%.4f" % fix.altitude,
        "navsat_status": int(fix.status.status),
        "position_covariance_type": int(fix.position_covariance_type),
        "position_covariance_xx": "%.9f" % position_covariance[0],
        "position_covariance_yy": "%.9f" % position_covariance[4],
        "position_covariance_zz": "%.9f" % position_covariance[8],
        "velocity_x_raw": "%.6f" % raw_velocity[0],
        "velocity_y_raw": "%.6f" % raw_velocity[1],
        "velocity_z_raw": "%.6f" % raw_velocity[2],
        "velocity_covariance_xx": "%.9f" % velocity_covariance[0],
        "velocity_covariance_yy": "%.9f" % velocity_covariance[7],
        "velocity_covariance_zz": "%.9f" % velocity_covariance[14],
        "p_lidar_x": "%.6f" % lidar_position[0],
        "p_lidar_y": "%.6f" % lidar_position[1],
        "p_lidar_z": "%.6f" % lidar_position[2],
        "v_lidar_x": "%.6f" % lidar_velocity[0],
        "v_lidar_y": "%.6f" % lidar_velocity[1],
        "v_lidar_z": "%.6f" % lidar_velocity[2],
        "speed_2d_m_s": "%.6f" % speed_2d,
        "heading_lidar_rad": "%.9f" % heading,
    }


def export_accumulated_dataset(args):
    """Stream a combined rosbag into an evaluation-friendly dataset."""
    from rtk_livox_dataset_tools.bag_utils import iter_deserialized_messages

    timing = _estimate_timing(args, iter_deserialized_messages)
    if os.path.exists(args.output_dir):
        raise RuntimeError(
            "Output directory already exists; choose a new path: %s"
            % args.output_dir
        )
    os.makedirs(os.path.join(args.output_dir, "velodyne"))
    os.makedirs(os.path.join(args.output_dir, "ImageSets"))
    calibration = _load_calibration(args.calib)
    accumulator = CausalCloudAccumulator(
        args.accumulation_sec, args.output_rate_hz
    )
    fix_stamps, fix_samples = [], []
    velocity_stamps, velocity_samples = [], []
    pending_clouds = []
    cloud_sequence = 0
    frames = []
    rtk_rows = []
    frame_ids = []
    skipped_no_rtk = 0
    stale_rtk_frames = 0
    dropped_stale_rtk = 0
    first_lidar_stamp = None

    def emit(candidate):
        nonlocal skipped_no_rtk, stale_rtk_frames, dropped_stale_rtk
        frame_stamp, window_start, packets = candidate
        if not packets:
            return
        latest_fix = _latest_at_or_before(
            fix_stamps, fix_samples, frame_stamp
        )
        latest_velocity = _latest_at_or_before(
            velocity_stamps, velocity_samples, frame_stamp
        )
        if latest_fix is None or latest_velocity is None:
            skipped_no_rtk += 1
            return
        fix_age = frame_stamp - latest_fix[0]
        velocity_age = frame_stamp - latest_velocity[0]
        if (
            fix_age < 0.0
            or velocity_age < 0.0
            or fix_age > args.max_rtk_age_sec
            or velocity_age > args.max_rtk_age_sec
        ):
            stale_rtk_frames += 1
            if args.drop_stale_rtk:
                dropped_stale_rtk += 1
                return

        raw_point_count = sum(len(points) for points in packets)
        points = reduce_points(
            np.concatenate(packets, axis=0),
            voxel_size=args.voxel_size,
            max_points=args.max_points_per_frame,
        )
        frame_id = len(frames)
        frame_name = "%06d" % frame_id
        relative_path = os.path.join("velodyne", frame_name + ".bin")
        points.tofile(os.path.join(args.output_dir, relative_path))
        frames.append(
            {
                "frame_id": frame_name,
                "file": relative_path,
                "stamp_sec": "%.9f" % frame_stamp,
                "relative_time_sec": "%.6f"
                % (frame_stamp - first_lidar_stamp),
                "window_start_sec": "%.9f" % window_start,
                "window_end_sec": "%.9f" % frame_stamp,
                "lidar_packet_count": len(packets),
                "raw_point_count": raw_point_count,
                "output_point_count": len(points),
            }
        )
        rtk_rows.append(
            _rtk_row(
                frame_id,
                frame_stamp,
                latest_fix,
                latest_velocity,
                calibration,
                args.max_rtk_age_sec,
            )
        )
        frame_ids.append(frame_name)

    topics = [args.cloud_topic, args.fix_topic, args.velocity_topic]
    for bag_message in iter_deserialized_messages(
        args.bag,
        topics=topics,
        storage_id=args.storage_id,
        use_header_stamp=False,
    ):
        if bag_message.topic == args.cloud_topic:
            if args.time_source == "aligned_header":
                cloud_stamp = (
                    _header_stamp_sec(bag_message.msg)
                    + timing["cloud_header_to_epoch_sec"]
                )
            else:
                cloud_stamp = bag_message.stamp_sec
            if first_lidar_stamp is None:
                first_lidar_stamp = cloud_stamp
            heapq.heappush(
                pending_clouds,
                (
                    cloud_stamp,
                    cloud_sequence,
                    pointcloud2_to_xyzi(bag_message.msg),
                ),
            )
            cloud_sequence += 1
        elif bag_message.topic == args.fix_topic:
            fix_stamp = (
                _header_stamp_sec(bag_message.msg)
                if args.time_source == "aligned_header"
                else bag_message.stamp_sec
            )
            _append_time_ordered(
                fix_stamps, fix_samples, fix_stamp, bag_message.msg
            )
        elif bag_message.topic == args.velocity_topic:
            velocity_stamp = (
                _header_stamp_sec(bag_message.msg)
                if args.time_source == "aligned_header"
                else bag_message.stamp_sec
            )
            _append_time_ordered(
                velocity_stamps,
                velocity_samples,
                velocity_stamp,
                bag_message.msg,
            )

        watermark = bag_message.stamp_sec - timing["reorder_slack_sec"]
        while pending_clouds and pending_clouds[0][0] <= watermark:
            cloud_stamp, _, points = heapq.heappop(pending_clouds)
            accumulator.add_cloud(
                cloud_stamp,
                points,
            )
        for candidate in accumulator.pop_due(watermark):
            emit(candidate)

    while pending_clouds:
        cloud_stamp, _, points = heapq.heappop(pending_clouds)
        accumulator.add_cloud(
            cloud_stamp,
            points,
        )
    if accumulator.last_cloud_stamp is not None:
        for candidate in accumulator.pop_through(accumulator.last_cloud_stamp):
            emit(candidate)
    if not frames:
        raise RuntimeError(
            "No synchronized frames were produced; check topics and "
            "RTK age limit"
        )

    frames_path = os.path.join(args.output_dir, "frames.csv")
    with open(frames_path, "w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=FRAME_FIELDS)
        writer.writeheader()
        writer.writerows(frames)
    with open(
        os.path.join(args.output_dir, "rtk_latest.csv"), "w", newline=""
    ) as file:
        writer = csv.DictWriter(file, fieldnames=RTK_FIELDS)
        writer.writeheader()
        writer.writerows(rtk_rows)
    split_path = os.path.join(args.output_dir, "ImageSets", "test.txt")
    with open(split_path, "w") as file:
        file.write("\n".join(frame_ids) + "\n")

    metadata = {
        "format": "rtk_livox_accumulated_openpcdet_v1",
        "source_bag": os.path.abspath(args.bag),
        "time_source": (
            "aligned_sensor_header" if args.time_source == "aligned_header"
            else "rosbag_record_timestamp"
        ),
        "timing": timing,
        "topics": {
            "cloud": args.cloud_topic,
            "fix": args.fix_topic,
            "velocity": args.velocity_topic,
        },
        "frame_policy": {
            "output_rate_hz": args.output_rate_hz,
            "accumulation_sec": args.accumulation_sec,
            "window": "(t - accumulation_sec, t]",
            "causal": True,
            "motion_compensation": "none; intended for a stationary LiDAR",
            "voxel_size": args.voxel_size,
            "max_points_per_frame": args.max_points_per_frame,
        },
        "rtk_policy": {
            "selection": "latest sample with timestamp <= frame timestamp",
            "interpolation": False,
            "max_age_sec": args.max_rtk_age_sec,
            "drop_stale_rtk": args.drop_stale_rtk,
            "calibration": os.path.abspath(args.calib) if args.calib else None,
        },
        "counts": {
            "frames": len(frames),
            "skipped_without_rtk": skipped_no_rtk,
            "frames_with_stale_rtk": stale_rtk_frames,
            "dropped_stale_rtk": dropped_stale_rtk,
            "total_output_points": int(
                sum(int(frame["output_point_count"]) for frame in frames)
            ),
        },
        "openpcdet": {
            "point_layout": ["x", "y", "z", "intensity"],
            "dtype": "float32",
            "intensity_policy": "preserved from Livox PointCloud2",
            "data_path": "velodyne",
            "frame_order": "frames.csv",
        },
        "evaluation_notes": [
            "RTK position is an antenna-point trajectory, not a 3D box "
            "label.",
            "Tracking metrics require antenna-to-object-center offset, "
            "class, and box dimensions.",
            "Use rtk_is_fresh and covariance fields to exclude unreliable "
            "GT samples.",
        ],
        "files": {
            "frames": "frames.csv",
            "rtk_latest": "rtk_latest.csv",
            "split": "ImageSets/test.txt",
        },
    }
    with open(os.path.join(args.output_dir, "metadata.yaml"), "w") as file:
        yaml.safe_dump(metadata, file, sort_keys=False)
    return metadata


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "Accumulate causal Livox windows and attach the latest RTK fix "
            "and velocity to every OpenPCDet frame."
        )
    )
    parser.add_argument("--bag", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--cloud-topic", default="/livox/lidar")
    parser.add_argument("--fix-topic", default="/ublox_gps_node/fix")
    parser.add_argument(
        "--velocity-topic", default="/ublox_gps_node/fix_velocity"
    )
    parser.add_argument("--storage-id", default="sqlite3")
    parser.add_argument(
        "--time-source",
        choices=["aligned_header", "bag"],
        default="aligned_header",
        help=(
            "Align the Livox relative header clock to the rosbag epoch and "
            "use RTK headers by default; 'bag' uses record timestamps only."
        ),
    )
    parser.add_argument("--output-rate-hz", type=float, default=10.0)
    parser.add_argument("--accumulation-sec", type=float, default=0.2)
    parser.add_argument("--max-rtk-age-sec", type=float, default=0.5)
    parser.add_argument(
        "--drop-stale-rtk",
        action="store_true",
        help=(
            "Drop frames whose latest RTK sample exceeds max age. By "
            "default frames are retained and marked rtk_is_fresh=0."
        ),
    )
    parser.add_argument("--voxel-size", type=float, default=0.0)
    parser.add_argument("--max-points-per-frame", type=int, default=0)
    parser.add_argument(
        "--calib",
        default="",
        help=(
            "Optional lidar/ENU calibration YAML. When supplied, RTK "
            "position and velocity are also written in the LiDAR frame."
        ),
    )
    args = parser.parse_args(argv)
    if args.max_rtk_age_sec < 0.0:
        parser.error("--max-rtk-age-sec must be non-negative")
    if args.voxel_size < 0.0:
        parser.error("--voxel-size must be non-negative")
    if args.max_points_per_frame < 0:
        parser.error("--max-points-per-frame must be non-negative")
    return args


def main(argv=None):
    args = parse_args(argv)
    metadata = export_accumulated_dataset(args)
    print("Wrote accumulated dataset: %s" % args.output_dir)
    print("frames: %d" % metadata["counts"]["frames"])
    print("points: %d" % metadata["counts"]["total_output_points"])
    print(
        "RTK (missing/stale/dropped): %d/%d/%d"
        % (
            metadata["counts"]["skipped_without_rtk"],
            metadata["counts"]["frames_with_stale_rtk"],
            metadata["counts"]["dropped_stale_rtk"],
        )
    )


if __name__ == "__main__":
    main()
