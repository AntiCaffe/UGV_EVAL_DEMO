from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _is_true(value):
    return value.strip().lower() in ("1", "true", "yes", "on")


def _exporter_node(context):
    arguments = [
        "--bag",
        LaunchConfiguration("bag").perform(context),
        "--output-dir",
        LaunchConfiguration("output_dir").perform(context),
        "--cloud-topic",
        LaunchConfiguration("cloud_topic").perform(context),
        "--fix-topic",
        LaunchConfiguration("fix_topic").perform(context),
        "--velocity-topic",
        LaunchConfiguration("velocity_topic").perform(context),
        "--storage-id",
        LaunchConfiguration("storage_id").perform(context),
        "--time-source",
        LaunchConfiguration("time_source").perform(context),
        "--output-rate-hz",
        LaunchConfiguration("output_rate_hz").perform(context),
        "--accumulation-sec",
        LaunchConfiguration("accumulation_sec").perform(context),
        "--max-rtk-age-sec",
        LaunchConfiguration("max_rtk_age_sec").perform(context),
        "--voxel-size",
        LaunchConfiguration("voxel_size").perform(context),
        "--max-points-per-frame",
        LaunchConfiguration("max_points_per_frame").perform(context),
    ]

    calib = LaunchConfiguration("calib").perform(context).strip()
    if calib:
        arguments.extend(["--calib", calib])

    if _is_true(LaunchConfiguration("drop_stale_rtk").perform(context)):
        arguments.append("--drop-stale-rtk")

    return [
        Node(
            package="rtk_livox_dataset_tools",
            executable="accumulated_bag_exporter",
            name="accumulated_bag_exporter",
            output="screen",
            emulate_tty=True,
            arguments=arguments,
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag", default_value="/project/bags/run_01_livox_rtk"
            ),
            DeclareLaunchArgument(
                "output_dir",
                default_value="/project/datasets/run_01_accumulated",
            ),
            DeclareLaunchArgument("cloud_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument(
                "fix_topic", default_value="/ublox_gps_node/fix"
            ),
            DeclareLaunchArgument(
                "velocity_topic",
                default_value="/ublox_gps_node/fix_velocity",
            ),
            DeclareLaunchArgument("storage_id", default_value="sqlite3"),
            DeclareLaunchArgument(
                "time_source", default_value="aligned_header"
            ),
            DeclareLaunchArgument("output_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("accumulation_sec", default_value="0.2"),
            DeclareLaunchArgument("max_rtk_age_sec", default_value="0.5"),
            DeclareLaunchArgument("drop_stale_rtk", default_value="false"),
            DeclareLaunchArgument("voxel_size", default_value="0.0"),
            DeclareLaunchArgument(
                "max_points_per_frame", default_value="0"
            ),
            DeclareLaunchArgument(
                "calib",
                default_value=(
                    "/project/calibration/run_01_lidar_rtk_alignment.yaml"
                ),
            ),
            OpaqueFunction(function=_exporter_node),
        ]
    )
