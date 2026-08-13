from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("output_dir", default_value="logs"),
            Node(
                package="rtk_livox_dataset_tools",
                executable="rtk_status_monitor",
                name="rtk_status_monitor",
                output="screen",
                arguments=["--output-dir", LaunchConfiguration("output_dir")],
            ),
        ]
    )
