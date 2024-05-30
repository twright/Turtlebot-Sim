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
1. Do 2D pose estimation (as described in the getting started guide). Observe that nothing happens.
1. In a separate terminal, source the environments: `source /opt/ros/humble/setup.zsh && source install/setup.zsh`
   - Assumes you are cd'ed into this root folder
1. Run `ros2 run scan_modifier scan_node`.
1. Define the area of the lidar that is not occluded, e.g., through the command: `ros2 topic pub --once /scan_config std_msgs/msg/Float32MultiArray "{data:[0.0, 3.14]}"`
   - This means that from 0.0 - 3.14 rad the lidar is not occluded.
1. Observe that the simulation now works as normal.

## Running the gradient costmap
1. Repeat steps 1 - 4 from the example above
1. Do 2D pose estimation (as described in the getting started guide).
1. Observe that the costmap in RViz is messed up - it is showing a gradient color instead of detecting obstacles


# Running on Turtlebot 4

## Simulation

### Spinning controller
Note that this is a work in progress
1. Build the navigation stack with the local nodes: `colcon build`
1. Patch the Turtlebot 4 config file which is found under `/opt/ros/humble/share/turtlebot4_navigation/config/nav2.yaml`:
   - The patch should apply similar changes as `spinning_controller.patch`
   - E.g.:
````yaml
# DWB parameters
FollowPath:
   plugin: "spinning_controller::DWBSpinningController" # In Iron and older versions, "/" was used instead of "::"
   spin_commands: [0.0, 0.0]
   spin_period: 5.0
   debug_trajectory_details: True
````
1. Run the simulation and see the behavior
   - Note that in the patch above it initialises the spin commands to 0


# Sources:
- Developing with the Nav2 stack: https://navigation.ros.org/
- Intercepting the /scan topic: https://husarion.com/tutorials/ros2-tutorials/2-creating-nodes-messages/#creating-a-publisher
