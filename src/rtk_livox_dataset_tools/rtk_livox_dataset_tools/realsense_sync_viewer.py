import argparse
import copy
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


FLOAT32_SIZE = 4


def _field_offset(cloud, name):
    for field in cloud.fields:
        if field.name == name:
            return field.offset, field.datatype
    return None, None


def _translated_cloud(cloud, frame_id, dx, dy, dz):
    out = copy.deepcopy(cloud)
    out.header.frame_id = frame_id
    offsets = {}
    for name in ("x", "y", "z"):
        offset, datatype = _field_offset(out, name)
        if offset is None:
            return out
        if datatype != PointField.FLOAT32:
            return out
        offsets[name] = offset

    shifts = {"x": dx, "y": dy, "z": dz}
    if dx == 0.0 and dy == 0.0 and dz == 0.0:
        return out

    data = bytearray(out.data)
    point_step = out.point_step
    point_count = out.width * out.height
    endian = ">" if out.is_bigendian else "<"
    fmt = endian + "f"
    for i in range(point_count):
        base = i * point_step
        for name, shift in shifts.items():
            if shift == 0.0:
                continue
            index = base + offsets[name]
            value = struct.unpack_from(fmt, data, index)[0]
            struct.pack_into(fmt, data, index, value + shift)
    out.data = bytes(data)
    return out


class RealsenseSyncViewer(Node):
    def __init__(self, args):
        super().__init__("realsense_sync_viewer")
        self.args = args
        self.pub1 = self.create_publisher(PointCloud2, args.output_cloud1_topic, 10)
        self.pub2 = self.create_publisher(PointCloud2, args.output_cloud2_topic, 10)
        self.create_subscription(PointCloud2, args.cloud1_topic, self._on_cloud1, 10)
        self.create_subscription(PointCloud2, args.cloud2_topic, self._on_cloud2, 10)
        self.get_logger().info(
            "Republishing %s and %s into frame %s"
            % (args.cloud1_topic, args.cloud2_topic, args.fixed_frame)
        )

    def _on_cloud1(self, msg):
        out = _translated_cloud(
            msg,
            self.args.fixed_frame,
            self.args.cloud1_offset_x,
            self.args.cloud1_offset_y,
            self.args.cloud1_offset_z,
        )
        self.pub1.publish(out)

    def _on_cloud2(self, msg):
        out = _translated_cloud(
            msg,
            self.args.fixed_frame,
            self.args.cloud2_offset_x,
            self.args.cloud2_offset_y,
            self.args.cloud2_offset_z,
        )
        self.pub2.publish(out)


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--cloud1-topic", default="/rs1/camera/depth/color/points")
    parser.add_argument("--cloud2-topic", default="/rs2/camera/depth/color/points")
    parser.add_argument("--output-cloud1-topic", default="/sync_check/rs1/points")
    parser.add_argument("--output-cloud2-topic", default="/sync_check/rs2/points")
    parser.add_argument("--fixed-frame", default="sync_check_world")
    parser.add_argument("--cloud1-offset-x", type=float, default=0.0)
    parser.add_argument("--cloud1-offset-y", type=float, default=0.0)
    parser.add_argument("--cloud1-offset-z", type=float, default=0.0)
    parser.add_argument("--cloud2-offset-x", type=float, default=0.0)
    parser.add_argument("--cloud2-offset-y", type=float, default=2.0)
    parser.add_argument("--cloud2-offset-z", type=float, default=0.0)
    return parser.parse_known_args(argv)


def main(argv=None):
    args, ros_args = parse_args(argv)
    rclpy.init(args=ros_args)
    node = RealsenseSyncViewer(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
