import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    package_dir = get_package_share_directory('qcar2_navigation')
    nav2_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(nav2_dir, 'launch')

    # Copy of the Cartographer launch file but adapted for navigation and map_server node.
    qcar2_cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('qcar2_navigation'), 'launch', 'qcar2_carto_nav_setup.launch.py')]
        )
    )

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    map_yaml_file = LaunchConfiguration('map')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(package_dir, 'config', 'qcar2_navigation_virtual.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file to load'
    )

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'
        ),

        # map_server node to load prebuilt map of the competition setup
        Node(
            condition=UnlessCondition(slam),
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[configured_params, {'yaml_filename': map_yaml_file}],
            remappings=remappings,
            output='screen',
        ),

        # lifecycle manager node to start map_server
        Node(
            condition=UnlessCondition(slam),
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['map_server']
            }]
        ),

        # Launch file for slam
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_launch.py')),
            condition=IfCondition(slam),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'use_respawn': use_respawn,
                              'params_file': params_file}.items()),

        # Launch file for nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),
    ])

    # Converter from nav2 commands to qcar2 commands for navigation
    qcar2_nav2_converter = Node(
        package='qcar2_nodes',
        executable='nav2_qcar2_converter',
        name='nav2_qcar2_converter',
    )

    motor_mux_node = Node(
        package='qcar2_controller',
        executable='motor_mux.py',
        parameters=[{'nav2_timeout': 3.0}]
    )

    line_detection = Node(
        package='qcar2_navigation',
        executable='lane_follower_pid.py',
    )

    stop_sign = Node(
        package='qcar2_controller',
        executable='stop_sign2.py'
    )

    redlight = Node(
        package='qcar2_controller',
        executable='redlight_detection.py'
    )

    # Launch rviz 
    rviz_navigation = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
            get_package_share_directory("qcar2_nodes"),
            "rviz",
            "rviz_navigation.rviz"
        )],
        # output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        qcar2_nav2_converter,
        qcar2_cartographer_launch,
        stdout_linebuf_envvar,
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        declare_slam_cmd,
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_use_composition_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,
        declare_map_yaml_cmd,
        bringup_cmd_group,
        motor_mux_node,
        line_detection,
        stop_sign,
        redlight,
        rviz_navigation,
    ])
