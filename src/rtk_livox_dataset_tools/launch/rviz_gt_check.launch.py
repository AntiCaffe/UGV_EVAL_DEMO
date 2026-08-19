from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _rviz_process(context):
    start_rviz = LaunchConfiguration("start_rviz").perform(context).strip().lower()
    if start_rviz not in ("1", "true", "yes", "on"):
        return []
    livox_frame = LaunchConfiguration("livox_frame").perform(context)
    return [
        ExecuteProcess(
            cmd=["rviz2", "--fixed-frame", livox_frame],
            output="screen",
            name="rviz2",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("calib", default_value="calibration/run_01_lidar_rtk_alignment.yaml"),
            DeclareLaunchArgument("time_offset_sec", default_value="0.0"),
            DeclareLaunchArgument("fix_topic", default_value="/ublox_gps_node/fix"),
            DeclareLaunchArgument("fix_velocity_topic", default_value="/ublox_gps_node/fix_velocity"),
            DeclareLaunchArgument("livox_frame", default_value="livox_frame"),
            DeclareLaunchArgument("marker_lifetime_sec", default_value="2.0"),
            DeclareLaunchArgument("show_speed_text", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            Node(
                package="rtk_livox_dataset_tools",
                executable="rtk_livox_visualizer",
                name="rtk_livox_visualizer",
                output="screen",
                emulate_tty=True,
                arguments=[
                    "--calib",
                    LaunchConfiguration("calib"),
                    "--time-offset-sec",
                    LaunchConfiguration("time_offset_sec"),
                    "--fix-topic",
                    LaunchConfiguration("fix_topic"),
                    "--fix-velocity-topic",
                    LaunchConfiguration("fix_velocity_topic"),
                    "--livox-frame",
                    LaunchConfiguration("livox_frame"),
                    "--marker-lifetime-sec",
                    LaunchConfiguration("marker_lifetime_sec"),
                    "--show-speed-text",
                    LaunchConfiguration("show_speed_text"),
                ],
            ),
            OpaqueFunction(function=_rviz_process),
        ]
    )
