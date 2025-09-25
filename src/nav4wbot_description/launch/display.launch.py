from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    description_dir = get_package_share_directory("nav4wbot_description")
    model_arg = DeclareLaunchArgument(
        name="model", 
        default_value=os.path.join(nav4wbot_description_dir, "urdf", "nav4wbot.urdf.xacro"),
        description="Absolute path to robot URDF file"
    )
    
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )
    
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )
    
    # Try to locate the .rviz config file; RViz will fallback to default if not found
    rviz_config = os.path.join(description_dir, "rviz", "nav4wbot.rviz")
    if os.path.isfile(rviz_config):
        rviz_args = ["-d", rviz_config]
    else:
        rviz_args = []

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=rviz_args
    )
    
    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])
