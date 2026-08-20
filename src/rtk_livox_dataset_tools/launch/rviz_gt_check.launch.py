from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _is_true(value):
    return value.strip().lower() in ("1", "true", "yes", "on")


def _rviz_process(context):
    if not _is_true(LaunchConfiguration("start_rviz").perform(context)):
        return []
    livox_frame = LaunchConfiguration("livox_frame").perform(context)
    return [
        ExecuteProcess(
            cmd=["rviz2", "--fixed-frame", livox_frame],
            output="screen",
            name="rviz2",
        )
    ]


def _bag_process(context):
    command = [
        "ros2",
        "bag",
        "play",
        LaunchConfiguration("bag").perform(context),
    ]
    if _is_true(LaunchConfiguration("loop").perform(context)):
        command.append("--loop")

    delay_sec = max(
        0.0, float(LaunchConfiguration("play_delay_sec").perform(context))
    )
    return [
        TimerAction(
            period=delay_sec,
            actions=[
                ExecuteProcess(
                    cmd=command,
                    output="screen",
                    name="rosbag_play",
                )
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag", default_value="/project/bags/run_01_livox_rtk"
            ),
            DeclareLaunchArgument("loop", default_value="true"),
            DeclareLaunchArgument("play_delay_sec", default_value="1.0"),
            DeclareLaunchArgument(
                "calib",
                default_value=(
                    "/project/calibration/run_01_lidar_rtk_alignment.yaml"
                ),
            ),
            DeclareLaunchArgument("time_offset_sec", default_value="0.0"),
            DeclareLaunchArgument(
                "fix_topic", default_value="/ublox_gps_node/fix"
            ),
            DeclareLaunchArgument(
                "fix_velocity_topic",
                default_value="/ublox_gps_node/fix_velocity",
            ),
            DeclareLaunchArgument("livox_frame", default_value="livox_frame"),
            DeclareLaunchArgument(
                "marker_lifetime_sec", default_value="2.0"
            ),
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
            OpaqueFunction(function=_bag_process),
        ]
    )
