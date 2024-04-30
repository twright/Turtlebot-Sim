# Overview
- This example changes certain behaviors of the Nav2 stack to allow us to implement the Turtlebot4 LiDAR occlusion example.
- It uses the Turtlebot 3 examples provided by the official ROS2 documentation.

# How to run the examples:
- The two examples can be run independently or in combination

## Running the /scan topic relay
1. Follow the instructions provided at https://navigation.ros.org/getting_started/index.html to ensure that you have a working Turtlebot 3 simulation environment.
1. Apply the patches: `./apply_topic_patch`
1. Build the updated Nav2 stack and other nodes: `colcon build`
1. Start the Turtlebot3 simulation: `./run_turtlebot3_sim.sh`
2. Do 2D pose estimation (as described in the getting started guide). Observe that nothing happens.
3. In a separate terminal, repeat the `source ...` commands from above and run `ros2 run scan_modifier scan_node`.
4. Observe that the simulation now works as normal.

## Running the gradient costmap
1. Repeat steps 1 - 4 from the example above
1. Do 2D pose estimation (as described in the getting started guide).
1. Observe that the costmap in RViz is messed up - it is showing a gradient color instead of detecting obstacles

# Sources:
- Developing with the Nav2 stack: https://navigation.ros.org/
- Intercepting the /scan topic: https://husarion.com/tutorials/ros2-tutorials/2-creating-nodes-messages/#creating-a-publisher
