import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_description = get_package_share_directory('nav4wbot_description')
    pkg_gazebo = get_package_share_directory('nav4wbot_gazebo')
    pkg_control = get_package_share_directory('nav4wbot_control')
    pkg_nav = get_package_share_directory('nav4wbot_navigation')

    # Include robot description launch
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'display.launch.py')
        )
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'world': os.path.join(pkg_gazebo, 'worlds', 'empty.world.sdf')}.items()
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_control, 'launch', 'control.launch.py')
        )
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={'params_file': os.path.join(pkg_nav, 'config', 'nav2_params.yaml')}.items()
    )

    return LaunchDescription([
        description_launch,
        gazebo_launch,
        control_launch,
        navigation_launch
    ])
