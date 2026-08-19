"""Publish an ordered KITTI velodyne sequence as PointCloud2 messages."""

import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


POINT_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(
        name='intensity', offset=12, datatype=PointField.FLOAT32, count=1
    ),
]
POINT_STEP = 4 * np.dtype(np.float32).itemsize


def find_default_velodyne_directory():
    """Find the source-tree kitti_samples symlink when no path is provided."""
    source_package = Path(__file__).resolve().parents[1]
    candidates = [
        source_package / 'kitti_samples',
        Path.cwd() / 'src' / 'pcdet_ros2' / 'kitti_samples',
        Path.cwd() / 'kitti_samples',
    ]
    for candidate in candidates:
        velodyne = candidate / 'velodyne'
        if velodyne.is_dir():
            return velodyne.resolve()
        if candidate.is_dir() and any(candidate.glob('*.bin')):
            return candidate.resolve()
    return None


def resolve_velodyne_directory(requested_path):
    """Resolve either a KITTI root or its velodyne directory."""
    if requested_path:
        candidate = Path(requested_path).expanduser().resolve()
        velodyne = candidate / 'velodyne'
        if velodyne.is_dir():
            return velodyne
        if candidate.is_dir():
            return candidate
        raise FileNotFoundError(
            f'KITTI dataset path does not exist: {candidate}'
        )

    velodyne = find_default_velodyne_directory()
    if velodyne is None:
        raise FileNotFoundError(
            'Could not find kitti_samples/velodyne automatically. Set the '
            'dataset_path parameter to the KITTI root or velodyne directory.'
        )
    return velodyne


def load_kitti_bin(path):
    """Load one KITTI [x, y, z, intensity] float32 point-cloud file."""
    values = np.fromfile(path, dtype=np.float32)
    if values.size % 4 != 0:
        raise ValueError(
            f'{path} contains {values.size} float32 values; expected a '
            'multiple of 4 for x, y, z, intensity'
        )
    points = values.reshape(-1, 4)
    return points[np.isfinite(points).all(axis=1)]


def points_to_message(points, stamp, frame_id):
    """Convert an Nx4 float array to a compact PointCloud2 message."""
    is_bigendian = sys.byteorder == 'big'
    byte_order = '>' if is_bigendian else '<'
    packed_points = np.ascontiguousarray(points, dtype=f'{byte_order}f4')

    message = PointCloud2()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = 1
    message.width = packed_points.shape[0]
    message.fields = POINT_FIELDS
    message.is_bigendian = is_bigendian
    message.point_step = POINT_STEP
    message.row_step = POINT_STEP * message.width
    message.data = packed_points.tobytes()
    message.is_dense = True
    return message


class KittiBinPublisher(Node):
    """Publish filename-ordered KITTI scans at a configurable fixed rate."""

    def __init__(self):
        super().__init__('kitti_bin_publisher')
        self.declare_parameter('dataset_path', '')
        self.declare_parameter('topic', '/livox/lidar')
        self.declare_parameter('frame_id', 'velodyne')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('loop', True)

        dataset_path = self.get_parameter('dataset_path').value
        topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.loop = bool(self.get_parameter('loop').value)

        if publish_rate_hz <= 0:
            raise ValueError('publish_rate_hz must be greater than zero')
        if not self.frame_id:
            raise ValueError('frame_id must not be empty')

        self.velodyne_directory = resolve_velodyne_directory(dataset_path)
        self.bin_files = sorted(self.velodyne_directory.glob('*.bin'))
        if not self.bin_files:
            raise FileNotFoundError(
                f'No .bin files found in {self.velodyne_directory}'
            )

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(PointCloud2, topic, qos)
        self.file_index = 0
        self.completed_loops = 0
        self.timer = self.create_timer(
            1.0 / publish_rate_hz, self.publish_next
        )

        self.get_logger().info(
            f'Publishing {len(self.bin_files)} KITTI scans from '
            f'{self.velodyne_directory} to {topic} at {publish_rate_hz:g} Hz '
            f'(frame_id={self.frame_id}, loop={self.loop})'
        )

    def publish_next(self):
        """Load and publish the next scan, wrapping when loop is enabled."""
        path = self.bin_files[self.file_index]
        try:
            points = load_kitti_bin(path)
        except (OSError, ValueError) as exc:
            self.get_logger().error(f'Failed to load {path}: {exc}')
            self.timer.cancel()
            return

        message = points_to_message(
            points=points,
            stamp=self.get_clock().now().to_msg(),
            frame_id=self.frame_id,
        )
        self.publisher.publish(message)
        self.get_logger().info(
            f'[{self.file_index + 1}/{len(self.bin_files)}] '
            f'{path.name}: {len(points)} points'
        )

        self.file_index += 1
        if self.file_index < len(self.bin_files):
            return

        if self.loop:
            self.file_index = 0
            self.completed_loops += 1
            self.get_logger().info(
                f'Restarting KITTI sequence (completed loops: '
                f'{self.completed_loops})'
            )
        else:
            self.timer.cancel()
            self.get_logger().info('KITTI sequence completed')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KittiBinPublisher()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
