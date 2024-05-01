#!/usr/bin/bash
source /opt/setup.bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=maze slam:=true nav2:=true rviz:=true