# Nav4wBot_ws
A 4 Wheel autonomous navigation bot.

# File Structure.
nav4wbot_ws/
└── src/
    ├── nav4wbot_description/   # URDF/Xacro, meshes, RViz config  -- cmake 
    ├── nav4wbot_gazebo/        # Gazebo world + robot sim plugins  -- cmake 
    ├── nav4wbot_control/       # ros2_control setup + diff_drive controller  -- cmake 
    ├── nav4wbot_bringup/       # Launch files for full system  -- python
    └── nav4wbot_navigation/    # Nav2 config (map, AMCL, planners)  -- python
# Demo Strucutre..
sim_ws/
  src/
    robot_description/                 # URDF/xacro + rviz config
      package.xml
      setup.cfg
      resource/
      urdf/
        four_wheel_robot.urdf.xacro
      launch/
        launch_sim.launch.py
      rviz/
        nav.rviz
    robot_control/                     # ros2_control + controllers
      package.xml
      config/
        controllers.yaml
      launch/
        controllers_launch.py
    robot_nav/                         # Nav2 params + helper script
      package.xml
      resource/
      config/
        nav2_params.yaml
      launch/
        nav_launch.py
      src/
        clicked_point_to_nav_goal.py

# POA
For now I am just adding all the files in src folder.. Note make pakages with pos2 create pkg and shift as per above demo strucutre.


# ROS Cmds 
colcon build
source install/setup.bash 
<!-- rviz -->
ros2 launch nav4wbot_description display.launch.py 
<!-- gazebo -->
ros2 launch nav4wbot_gazebo gz_sim.launch.py
