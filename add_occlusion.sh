#!/usr/bin/env bash
sudo ln -f -s /ws/src/models/meshes/occlusion$@.dae /opt/ros/humble/share/turtlebot4_description/meshes/occlusion.dae
sudo ln -f -s /ws/src/models/urdf/rplidar_occluded.xacro /opt/ros/humble/share/turtlebot4_description/urdf/sensors/rplidar.urdf.xacro