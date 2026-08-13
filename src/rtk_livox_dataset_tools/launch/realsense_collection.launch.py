from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _is_true(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _actions(context):
    actions = []
    camera_namespace = LaunchConfiguration("camera_namespace").perform(context)
    camera_name = LaunchConfiguration("camera_name").perform(context)

    if _is_true(LaunchConfiguration("start_camera").perform(context)):
        actions.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "launch",
                    LaunchConfiguration("realsense_package").perform(context),
                    LaunchConfiguration("realsense_launch_file").perform(context),
                    "camera_namespace:=%s" % camera_namespace,
                    "camera_name:=%s" % camera_name,
                    "enable_depth:=true",
                    "enable_color:=true",
                    "align_depth.enable:=true",
                    "pointcloud.enable:=true",
                ],
                output="screen",
                name="realsense_launch",
            )
        )

    topics_text = LaunchConfiguration("record_topics").perform(context)
    topics = [topic for topic in topics_text.split() if topic]
    if not topics:
        topics = ["/%s/%s/depth/color/points" % (camera_namespace, camera_name)]

    cmd = ["ros2", "bag", "record", "-o", LaunchConfiguration("bag_uri").perform(context)]
    cmd.extend(topics)
    actions.append(ExecuteProcess(cmd=cmd, output="screen", name="realsense_bag_record"))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag_uri", default_value="bags/run_01_rs1"),
            DeclareLaunchArgument("camera_namespace", default_value="rs1"),
            DeclareLaunchArgument("camera_name", default_value="camera"),
            DeclareLaunchArgument(
                "record_topics",
                default_value="/rs1/camera/depth/color/points",
                description="Whitespace-separated topics to record. Override for rs2.",
            ),
            DeclareLaunchArgument("start_camera", default_value="true"),
            DeclareLaunchArgument("realsense_package", default_value="realsense2_camera"),
            DeclareLaunchArgument("realsense_launch_file", default_value="rs_launch.py"),
            OpaqueFunction(function=_actions),
        ]
    )
