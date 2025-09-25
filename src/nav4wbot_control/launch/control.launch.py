import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_control_dir = get_package_share_directory('nav4wbot_control')
    
    # Allow passing controller config file via launch argument (default to diff_drive_controllers.yaml)
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_control_dir, 'config', 'diff_drive_controllers.yaml'),
        description='Path to ros2_control controller configuration yaml file'
    )
    
    config_file = LaunchConfiguration('config_file')
    
    # Start ros2_control_node with controller configuration
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[config_file],
        output='screen'
    )
    
    # Spawn the diff_drive_controller
    spawn_diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
    )
    
    return LaunchDescription([
        config_file_arg,
        ros2_control_node,
        spawn_diff_drive_controller
    ])