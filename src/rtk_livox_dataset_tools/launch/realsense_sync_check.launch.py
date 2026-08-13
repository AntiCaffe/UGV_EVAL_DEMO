from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _rviz_process(context):
    start_rviz = LaunchConfiguration("start_rviz").perform(context).strip().lower()
    if start_rviz not in ("1", "true", "yes", "on"):
        return []
    return [ExecuteProcess(cmd=["rviz2"], output="screen", name="rviz2")]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("cloud1_topic", default_value="/rs1/camera/depth/color/points"),
            DeclareLaunchArgument("cloud2_topic", default_value="/rs2/camera/depth/color/points"),
            DeclareLaunchArgument("fixed_frame", default_value="sync_check_world"),
            DeclareLaunchArgument("cloud2_offset_y", default_value="2.0"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            Node(
                package="rtk_livox_dataset_tools",
                executable="realsense_sync_viewer",
                name="realsense_sync_viewer",
                output="screen",
                arguments=[
                    "--cloud1-topic",
                    LaunchConfiguration("cloud1_topic"),
                    "--cloud2-topic",
                    LaunchConfiguration("cloud2_topic"),
                    "--fixed-frame",
                    LaunchConfiguration("fixed_frame"),
                    "--cloud2-offset-y",
                    LaunchConfiguration("cloud2_offset_y"),
                ],
            ),
            OpaqueFunction(function=_rviz_process),
        ]
    )
