"""Launch the KITTI binary point-cloud sequence publisher."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    dataset_path = LaunchConfiguration('dataset_path')
    input_topic = LaunchConfiguration('input_topic')
    frame_id = LaunchConfiguration('frame_id')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    loop = LaunchConfiguration('loop')

    return LaunchDescription([
        DeclareLaunchArgument(
            'dataset_path',
            default_value='',
            description=(
                'KITTI root or velodyne directory. Empty uses the '
                'pcdet_ros2/kitti_samples symlink.'
            ),
        ),
        DeclareLaunchArgument(
            'input_topic',
            default_value='/livox/lidar',
            description='PointCloud2 topic consumed by pcdet_ros2',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='velodyne',
            description='Frame ID assigned to published point clouds',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='10.0',
            description='Point-cloud playback rate in Hz',
        ),
        DeclareLaunchArgument(
            'loop',
            default_value='true',
            choices=['true', 'false'],
            description='Restart at the first scan after the final scan',
        ),
        Node(
            package='pcdet_ros2',
            executable='kitti_bin_publisher',
            name='kitti_bin_publisher',
            output='screen',
            parameters=[{
                'dataset_path': dataset_path,
                'topic': input_topic,
                'frame_id': frame_id,
                'publish_rate_hz': ParameterValue(
                    publish_rate_hz, value_type=float
                ),
                'loop': ParameterValue(loop, value_type=bool),
            }],
        ),
    ])
