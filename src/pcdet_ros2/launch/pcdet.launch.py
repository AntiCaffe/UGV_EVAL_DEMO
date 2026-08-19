import os


from ament_index_python.packages import get_package_share_directory


# ros2 pkg 생성하면서 작성되는 코드
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml



def generate_launch_description():
    package_name = 'pcdet_ros2'
    package_dir = get_package_share_directory(package_name)

    # Custom file 
    # config_file = 'pcdet_second.param.yaml'
    config_file = 'pcdet_center.param.yaml'

    # Basic file
    # config_file = 'pcdet_centerpont_pillar_param.yaml'
    # config_file = 'pcdet_GLE.param.yaml'
    # config_file = 'pcdet_hednet.param.yaml'
    # config_file = 'pcdet_parta2_free.param.yaml'
    # config_file = 'pcdet_parta2.param.yaml'
    # config_file = 'pcdet_pointrcnn.param.yaml'
    # config_file = 'pcdet_pvrcnn.param.yaml'
    # config_file = 'pcdet_pointpillar.param.yaml'

    


    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    output_format = LaunchConfiguration('output_format')

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={}
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(package_dir, 'config', config_file),
        description='Full path to the ROS 2 parameters file to use for the launched nodes'
    )

    declare_input_topic_cmd = DeclareLaunchArgument(
        'input_topic',
        default_value='/livox/lidar',
        # default_value='/ouster/points2',
        description='Input Point Cloud'
    )

    declare_output_topic_cmd = DeclareLaunchArgument(
        'output_topic',
        default_value='lr_detections',
        description='Output object detections topic'
    )

    declare_output_format_cmd = DeclareLaunchArgument(
        'output_format',
        default_value='marker_array',
        choices=['marker_array', 'detection3d_array'],
        description='Output type: MarkerArray or Detection3DArray'
    )

    pcdet = Node(
        package=package_name,
        executable='pcdet',
        name='pcdet',
        output='screen',
        parameters=[configured_params,
                    {'package_folder_path': package_dir,
                     'output_format': output_format}],
        remappings=[("input", input_topic),
                    ("output", output_topic)]
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_input_topic_cmd)
    ld.add_action(declare_output_topic_cmd)
    ld.add_action(declare_output_format_cmd)
    ld.add_action(pcdet)

    return ld
