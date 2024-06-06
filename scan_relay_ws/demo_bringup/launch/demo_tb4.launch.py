from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    ld = LaunchDescription([])

    demo_bringup_dir = get_package_share_directory("demo_bringup")
    tb4_simulation_launch_file = PathJoinSubstitution(
        [demo_bringup_dir, "launch", "other", "turtlebot4_ignition.launch.py"]
    )

    tb4_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb4_simulation_launch_file),
        launch_arguments=[
            ("world", "maze"),
            ("slam", "true"),
            ("nav2", "true"),
            ("rviz", "true"),
        ],
    )

    scan_node = Node(
        package="scan_modifier",
        executable="scan_node",
    )

    spin_config_node = Node(
        package="topic_param_bridge",
        executable="param_bridge",
    )

    ld.add_action(tb4_ld)
    ld.add_action(scan_node)
    ld.add_action(spin_config_node)

    return ld
