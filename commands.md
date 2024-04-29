# Useful ROS2 commands to remember

- Read IP: ros2 topic echo /ip
- Read LiDAR data raw: ros2 topic echo /scan
- See topics being published: ros2 topic list
- Undock robot: ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
- Dock robot: ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"