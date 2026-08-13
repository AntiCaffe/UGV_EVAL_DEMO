from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _is_true(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _processes(context):
    actions = []

    if _is_true(LaunchConfiguration("start_ublox").perform(context)):
        actions.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "launch",
                    LaunchConfiguration("ublox_package").perform(context),
                    LaunchConfiguration("ublox_launch_file").perform(context),
                ],
                output="screen",
                name="ublox_launch",
            )
        )

    if _is_true(LaunchConfiguration("start_c099_udp").perform(context)):
        actions.append(
            Node(
                package="rtk_livox_dataset_tools",
                executable="c099_udp_bridge",
                name="ublox_gps_node",
                output="screen",
                arguments=[
                    "--host",
                    LaunchConfiguration("c099_host"),
                    "--port",
                    LaunchConfiguration("c099_port"),
                    "--frame-id",
                    LaunchConfiguration("frame_id"),
                    "--navpvt-topic",
                    "/ublox_gps_node/navpvt",
                    "--fix-topic",
                    "/ublox_gps_node/fix",
                    "--fix-velocity-topic",
                    "/ublox_gps_node/fix_velocity",
                    "--rtcm-topic",
                    "/rtcm",
                    "--raw-log",
                    LaunchConfiguration("c099_raw_log"),
                    "--configure-rate-hz",
                    LaunchConfiguration("c099_configure_rate_hz"),
                    "--forward-rtcm",
                ],
            )
        )

    if _is_true(LaunchConfiguration("start_ntrip").perform(context)):
        cmd = [
            "ros2",
            "launch",
            LaunchConfiguration("ntrip_package").perform(context),
            LaunchConfiguration("ntrip_launch_file").perform(context),
            "host:=%s" % LaunchConfiguration("ntrip_host").perform(context),
            "port:=%s" % LaunchConfiguration("ntrip_port").perform(context),
            "mountpoint:=%s" % LaunchConfiguration("ntrip_mountpoint").perform(context),
            "authenticate:=%s" % LaunchConfiguration("ntrip_authenticate").perform(context),
            "username:=%s" % LaunchConfiguration("ntrip_username").perform(context),
            "password:=%s" % LaunchConfiguration("ntrip_password").perform(context),
            "rtcm_message_package:=%s" % LaunchConfiguration("rtcm_message_package").perform(context),
        ]
        actions.append(ExecuteProcess(cmd=cmd, output="screen", name="ntrip_launch"))

    if _is_true(LaunchConfiguration("start_bag").perform(context)):
        bag_uri = LaunchConfiguration("bag_uri").perform(context)
        topics_text = LaunchConfiguration("record_topics").perform(context)
        topics = [topic for topic in topics_text.split() if topic]
        if not topics:
            topics = [
                "/ublox_gps_node/navpvt",
                "/ublox_gps_node/fix",
                "/ublox_gps_node/fix_velocity",
                "/rtcm",
            ]

        cmd = ["ros2", "bag", "record", "-o", bag_uri]
        cmd.extend(topics)
        actions.append(ExecuteProcess(cmd=cmd, output="screen", name="rtk_bag_record"))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag_uri", default_value="bags/run_01_rtk"),
            DeclareLaunchArgument(
                "record_topics",
                default_value="/ublox_gps_node/navpvt /ublox_gps_node/fix /ublox_gps_node/fix_velocity /rtcm",
            ),
            DeclareLaunchArgument("start_bag", default_value="true"),
            DeclareLaunchArgument("start_ublox", default_value="true"),
            DeclareLaunchArgument("ublox_package", default_value="ublox_gps"),
            DeclareLaunchArgument("ublox_launch_file", default_value="ublox_gps_node-launch.py"),
            DeclareLaunchArgument("start_c099_udp", default_value="false"),
            DeclareLaunchArgument("c099_host", default_value="192.168.0.1"),
            DeclareLaunchArgument("c099_port", default_value="5555"),
            DeclareLaunchArgument("c099_raw_log", default_value="rtk_udp_log.bin"),
            DeclareLaunchArgument("c099_configure_rate_hz", default_value="0.0"),
            DeclareLaunchArgument("frame_id", default_value="gps"),
            DeclareLaunchArgument("start_ntrip", default_value="true"),
            DeclareLaunchArgument("ntrip_package", default_value="ntrip_client"),
            DeclareLaunchArgument("ntrip_launch_file", default_value="ntrip_client_launch.py"),
            DeclareLaunchArgument("ntrip_host", default_value="www.gnssdata.or.kr"),
            DeclareLaunchArgument("ntrip_port", default_value="2101"),
            DeclareLaunchArgument("ntrip_mountpoint", default_value=""),
            DeclareLaunchArgument("ntrip_authenticate", default_value="True"),
            DeclareLaunchArgument("ntrip_username", default_value=""),
            DeclareLaunchArgument("ntrip_password", default_value=""),
            DeclareLaunchArgument("rtcm_message_package", default_value="rtcm_msgs"),
            DeclareLaunchArgument("status_output_dir", default_value="logs"),
            Node(
                package="rtk_livox_dataset_tools",
                executable="rtk_status_monitor",
                name="rtk_status_monitor",
                output="screen",
                arguments=["--output-dir", LaunchConfiguration("status_output_dir")],
            ),
            OpaqueFunction(function=_processes),
        ]
    )
