import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_nav = get_package_share_directory('nav4wbot_navigation')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_nav, 'config', 'nav2_params.yaml'),
        description='Full path to config file to use for all launched nodes'
    )

    params_file = LaunchConfiguration('params_file')
    map_yaml_file = os.path.join(pkg_nav, 'maps', 'test_map.yaml')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_yaml_file, 'use_sim_time': True}],
        output='screen'
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen'
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[params_file],
        output='screen'
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[params_file],
        output='screen'
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[params_file],
        output='screen'
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl', 'planner_server', 'controller_server', 'bt_navigator']}]
    )

    return LaunchDescription([
        params_file_arg,
        map_server,
        amcl,
        planner_server,
        controller_server,
        bt_navigator,
        lifecycle_manager
    ])
