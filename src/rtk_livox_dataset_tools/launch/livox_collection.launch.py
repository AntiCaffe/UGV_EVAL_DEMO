from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _record_process(context):
    bag_uri = LaunchConfiguration("bag_uri").perform(context)
    topics_text = LaunchConfiguration("record_topics").perform(context)
    topics = [topic for topic in topics_text.split() if topic]
    if not topics:
        topics = ["/livox/lidar"]

    cmd = ["ros2", "bag", "record", "-o", bag_uri]
    cmd.extend(topics)
    return [
        ExecuteProcess(
            cmd=cmd,
            output="screen",
            name="livox_bag_record",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_uri",
                default_value="bags/run_01_livox",
                description="Output rosbag URI for the Livox laptop.",
            ),
            DeclareLaunchArgument(
                "record_topics",
                default_value="/livox/lidar /tracking/objects /tracking/tracks",
                description="Whitespace-separated Livox/perception topics to record.",
            ),
            OpaqueFunction(function=_record_process),
        ]
    )
