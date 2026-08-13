import argparse
import math
import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PointStamped, TwistStamped, TwistWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray

from rtk_livox_dataset_tools.geo import llh_to_enu
from rtk_livox_dataset_tools.transforms import rotation_from_yaw


def _load_calibration(path):
    path = _resolve_calibration_path(path)
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    required = ["origin_llh", "yaw_enu_lidar_rad", "lidar_position_enu"]
    missing = [key for key in required if key not in data]
    if missing:
        raise ValueError("Calibration YAML is missing: %s" % ", ".join(missing))
    return data


def _resolve_calibration_path(path):
    if os.path.isabs(path) or os.path.exists(path):
        return path

    candidates = []
    try:
        prefix = get_package_prefix("rtk_livox_dataset_tools")
        install_dir = os.path.dirname(prefix)
        workspace_root = os.path.dirname(install_dir)
        candidates.append(os.path.join(workspace_root, path))
    except Exception:
        pass

    candidates.extend(
        [
            os.path.join(os.getcwd(), path),
            os.path.join(os.path.expanduser("~/workspaces/sensor_project_dataset_2026_ws"), path),
        ]
    )
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    raise FileNotFoundError(
        "Calibration file not found: %s. Tried: %s"
        % (path, ", ".join(candidates) if candidates else path)
    )


def _offset_stamp(stamp, offset_sec):
    total_nsec = int(stamp.sec) * 1000000000 + int(stamp.nanosec)
    total_nsec += int(round(offset_sec * 1000000000.0))
    if total_nsec < 0:
        total_nsec = 0
    out = Time()
    out.sec = total_nsec // 1000000000
    out.nanosec = total_nsec % 1000000000
    return out


def _str_to_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class RtkLivoxVisualizer(Node):
    def __init__(self, args):
        super().__init__("rtk_livox_visualizer")
        self.args = args
        calib = _load_calibration(args.calib)
        self.origin_llh = [float(v) for v in calib["origin_llh"]]
        self.p_lidar_enu = np.asarray(calib["lidar_position_enu"], dtype=float)
        self.r_lidar_enu = rotation_from_yaw(float(calib["yaw_enu_lidar_rad"])).T
        calib_antenna_offset = calib.get("p_antenna_in_lidar", [0.0, 0.0, 0.0])
        self.declare_parameter(
            "p_antenna_in_lidar_x",
            float(args.p_antenna_in_lidar_x if args.p_antenna_in_lidar_x is not None else calib_antenna_offset[0]),
        )
        self.declare_parameter(
            "p_antenna_in_lidar_y",
            float(args.p_antenna_in_lidar_y if args.p_antenna_in_lidar_y is not None else calib_antenna_offset[1]),
        )
        self.declare_parameter(
            "p_antenna_in_lidar_z",
            float(args.p_antenna_in_lidar_z if args.p_antenna_in_lidar_z is not None else calib_antenna_offset[2]),
        )
        self.latest_velocity = None

        self.point_pub = self.create_publisher(PointStamped, args.output_point_topic, 10)
        self.velocity_pub = self.create_publisher(TwistStamped, args.output_velocity_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, args.output_marker_topic, 10)

        self.create_subscription(NavSatFix, args.fix_topic, self._on_fix, 10)
        self.create_subscription(
            TwistWithCovarianceStamped,
            args.fix_velocity_topic,
            self._on_velocity,
            10,
        )

        self.get_logger().info(
            "Publishing RTK GT in %s from %s and %s"
            % (args.livox_frame, args.fix_topic, args.fix_velocity_topic)
        )

    def _on_velocity(self, msg):
        self.latest_velocity = msg

    def _position_lidar(self, fix):
        p_enu = np.asarray(
            llh_to_enu(fix.latitude, fix.longitude, fix.altitude, self.origin_llh),
            dtype=float,
        )
        p_antenna_lidar = self.r_lidar_enu.dot(p_enu - self.p_lidar_enu)
        return p_antenna_lidar - self._antenna_offset_lidar()

    def _antenna_offset_lidar(self):
        return np.asarray(
            [
                self.get_parameter("p_antenna_in_lidar_x").value,
                self.get_parameter("p_antenna_in_lidar_y").value,
                self.get_parameter("p_antenna_in_lidar_z").value,
            ],
            dtype=float,
        )

    def _velocity_lidar(self):
        if self.latest_velocity is None:
            return np.zeros(3, dtype=float)
        linear = self.latest_velocity.twist.twist.linear
        v_enu = np.asarray([linear.x, linear.y, linear.z], dtype=float)
        return self.r_lidar_enu.dot(v_enu)

    def _on_fix(self, msg):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude) or math.isnan(msg.altitude):
            return
        p_lidar = self._position_lidar(msg)
        v_lidar = self._velocity_lidar()
        header_stamp = _offset_stamp(msg.header.stamp, self.args.time_offset_sec)

        point = PointStamped()
        point.header.stamp = header_stamp
        point.header.frame_id = self.args.livox_frame
        point.point.x = float(p_lidar[0])
        point.point.y = float(p_lidar[1])
        point.point.z = float(p_lidar[2])
        self.point_pub.publish(point)

        twist = TwistStamped()
        twist.header = point.header
        twist.twist.linear.x = float(v_lidar[0])
        twist.twist.linear.y = float(v_lidar[1])
        twist.twist.linear.z = float(v_lidar[2])
        self.velocity_pub.publish(twist)

        self.marker_pub.publish(self._markers(point, v_lidar))

    def _markers(self, point, v_lidar):
        cylinder = Marker()
        cylinder.header = point.header
        cylinder.ns = "rtk_gt"
        cylinder.id = 1
        cylinder.type = Marker.CYLINDER
        cylinder.action = Marker.ADD
        cylinder.pose.position.x = point.point.x
        cylinder.pose.position.y = point.point.y
        cylinder_mid_z = point.point.z + 0.5 * self.args.cylinder_height
        cylinder.pose.position.z = cylinder_mid_z
        cylinder.pose.orientation.w = 1.0
        cylinder.scale.x = self.args.cylinder_diameter
        cylinder.scale.y = self.args.cylinder_diameter
        cylinder.scale.z = self.args.cylinder_height
        cylinder.color.r = 0.0
        cylinder.color.g = 0.85
        cylinder.color.b = 0.2
        cylinder.color.a = 0.65
        cylinder.lifetime.sec = 0
        cylinder.lifetime.nanosec = int(self.args.marker_lifetime_sec * 1000000000.0)

        arrow = Marker()
        arrow.header = point.header
        arrow.ns = "rtk_gt"
        arrow.id = 2
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.orientation.w = 1.0
        arrow.color.r = 1.0
        arrow.color.g = 0.25
        arrow.color.b = 0.0
        arrow.color.a = 0.9
        arrow.lifetime = cylinder.lifetime

        speed = math.hypot(float(v_lidar[0]), float(v_lidar[1]))
        length = self.args.arrow_length
        diameter = min(
            self.args.arrow_max_diameter,
            max(self.args.arrow_base_diameter, speed * self.args.arrow_diameter_per_mps),
        )
        arrow.scale.x = diameter
        arrow.scale.y = diameter * 2.5
        arrow.scale.z = max(0.15, diameter * 4.0)

        start = Point()
        start.x = point.point.x
        start.y = point.point.y
        start.z = cylinder_mid_z + self.args.arrow_z_offset
        end = Point()
        if speed > 1.0e-3:
            direction = v_lidar[:2] / speed
            end.x = start.x + float(direction[0]) * length
            end.y = start.y + float(direction[1]) * length
        else:
            end.x = start.x
            end.y = start.y
        end.z = start.z
        arrow.points = [start, end]

        markers = [cylinder, arrow]
        if self.args.show_speed_text:
            text = Marker()
            text.header = point.header
            text.ns = "rtk_gt"
            text.id = 3
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = point.point.x
            text.pose.position.y = point.point.y
            text.pose.position.z = cylinder_mid_z + self.args.speed_text_z_offset
            text.pose.orientation.w = 1.0
            text.scale.z = self.args.speed_text_height
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 0.95
            text.lifetime = cylinder.lifetime
            text.text = "speed: %.2f m/s" % speed
            markers.append(text)

        return MarkerArray(markers=markers)


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--calib", required=True)
    parser.add_argument("--fix-topic", default="/ublox_gps_node/fix")
    parser.add_argument("--fix-velocity-topic", default="/ublox_gps_node/fix_velocity")
    parser.add_argument("--output-point-topic", default="/rtk_gt/livox/point")
    parser.add_argument("--output-velocity-topic", default="/rtk_gt/livox/velocity")
    parser.add_argument("--output-marker-topic", default="/rtk_gt/livox/markers")
    parser.add_argument("--livox-frame", default="livox_frame")
    parser.add_argument("--time-offset-sec", type=float, default=0.0)
    parser.add_argument("--p-antenna-in-lidar-x", type=float)
    parser.add_argument("--p-antenna-in-lidar-y", type=float)
    parser.add_argument("--p-antenna-in-lidar-z", type=float)
    parser.add_argument("--cylinder-diameter", type=float, default=0.45)
    parser.add_argument("--cylinder-height", type=float, default=1.7)
    parser.add_argument("--arrow-z-offset", type=float, default=0.0)
    parser.add_argument("--arrow-length", type=float, default=1.5)
    parser.add_argument("--arrow-base-diameter", type=float, default=0.04)
    parser.add_argument("--arrow-diameter-per-mps", type=float, default=0.035)
    parser.add_argument("--arrow-max-diameter", type=float, default=0.18)
    parser.add_argument("--show-speed-text", type=_str_to_bool, default=True)
    parser.add_argument("--speed-text-height", type=float, default=0.35)
    parser.add_argument("--speed-text-z-offset", type=float, default=0.8)
    parser.add_argument("--marker-lifetime-sec", type=float, default=0.4)
    return parser.parse_known_args(argv)


def main(argv=None):
    args, ros_args = parse_args(argv)
    rclpy.init(args=ros_args)
    node = RtkLivoxVisualizer(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
