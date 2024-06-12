from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

TURTLEBOT4_REAL_SCAN_SIZE = 1080


def generate_launch_description():
    ld = LaunchDescription([])

    demo_bringup_dir = get_package_share_directory("demo_bringup")

    localization_launch_file = PathJoinSubstitution(
        [
            get_package_share_directory("turtlebot4_navigation"),
            "launch",
            "localization.launch.py",
        ]
    )

    nav2_launch_file = PathJoinSubstitution(
        [
            get_package_share_directory("turtlebot4_navigation"),
            "launch",
            "nav2.launch.py",
        ]
    )

    params_path = PathJoinSubstitution([demo_bringup_dir, "config", "tb4_nav2.yaml"])
    map_path = PathJoinSubstitution([demo_bringup_dir, "maps", "office.yaml"])

    nav2_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments=[("params_file", params_path)],
    )

    localization_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_file),
        launch_arguments=[("map", map_path)],
    )

    scan_node = Node(
        package="scan_modifier",
        executable="scan_node",
        parameters=[{"scan_ranges_size": TURTLEBOT4_REAL_SCAN_SIZE}],
    )

    spin_config_node = Node(
        package="topic_param_bridge",
        executable="param_bridge",
    )

    ld.add_action(localization_ld)
    ld.add_action(nav2_ld)
    ld.add_action(scan_node)
    # ld.add_action(spin_config_node)

    return ld
