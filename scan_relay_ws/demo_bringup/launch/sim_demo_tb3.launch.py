from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription(
        [
            SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value="waffle"),
            SetEnvironmentVariable(
                name="GAZEBO_MODEL_PATH",
                value="/opt/ros/humble/share/turtlebot3_gazebo/models",
            ),
        ]
    )

    nav2_launch_file = os.path.join(
        get_package_share_directory("nav2_bringup"),
        "launch/tb3_simulation_launch.py",
    )

    nav2_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={"headless": "False"}.items(),
    )

    scan_node = Node(
        package="scan_modifier",
        executable="scan_node",
    )

    spin_config_node = Node(
        package="topic_param_bridge",
        executable="param_bridge",
    )

    ld.add_action(nav2_ld)
    ld.add_action(scan_node)
    ld.add_action(spin_config_node)

    return ld
