from collections import namedtuple
import subprocess

import rclpy
from rclpy.node import Node

from spin_interfaces.msg import SpinPeriodicCommands


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(
            SpinPeriodicCommands, "/spin_config", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Heard: "{msg}"')

        spin_commands = [
            value for cmd in msg.commands for value in (cmd.omega, cmd.duration)
        ]
        spin_period = msg.period

        self.get_logger().info(f'Sending: "{spin_commands}" and "{spin_period}"')
        cmd_str = f"""ros2 param set /controller_server FollowPath.spin_commands "{spin_commands}" """
        subprocess.run(cmd_str, shell=True)
        cmd_str = f"""ros2 param set /controller_server FollowPath.spin_period {spin_period}"""
        subprocess.run(cmd_str, shell=True)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
