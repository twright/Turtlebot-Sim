# Overview
- This example changes certain behaviors of the Nav2 stack to allow us to implement the Turtlebot4 LiDAR occlusion example.
- It uses the Turtlebot 3 examples provided by the official ROS2 documentation.

# How to run this example
- Follow the instructions provided at https://navigation.ros.org/getting_started/index.html to ensure that you have a working Turtlebot 3 simulation environment.
- Apply the patches: `./apply_patches`
- Build the updated Nav2 stack and other nodes: `colcon build`
- Source global ROS2 settings: `source /opt/ros/humble/setup.zsh`
- Source workspace settings: `source install/setup.zsh`
- Export Turtlebot version: `export TURTLEBOT3_MODEL=waffle`
- Export Gazebo model path: `export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models`
- Start the simulation: `ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False`
- Do 2D pose estimation (as described in the getting started guide). Observe that nothing happens.
- In a separate terminal, repeat the `source ...` commands from above and run `ros2 run scan_modifier scan_node`.
- Observe that the simulation now works as normal.

# Sources:
- Developing with the Nav2 stack: https://navigation.ros.org/
- Intercepting the /scan topic: https://husarion.com/tutorials/ros2-tutorials/2-creating-nodes-messages/#creating-a-publisher