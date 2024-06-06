import rclpy
from rclpy.node import Node

from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from spin_interfaces.msg import SpinPeriodicCommands


class TopicParamBridge(Node):

    def __init__(
        self,
        service_name="/controller_server/set_parameters",
        spin_period_param_name="FollowPath.spin_period",
        spin_commands_param_name="FollowPath.spin_commands",
        spin_config_topic="/spin_config",
    ):
        super().__init__("topic_param_bridge")
        self.service_name = service_name
        self.spin_period_param_name = spin_period_param_name
        self.spin_commands_param_name = spin_commands_param_name
        self.spin_config_topic = spin_config_topic

        self.cli = self.create_client(SetParameters, self.service_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = SetParameters.Request()

        self.subscription = self.create_subscription(
            SpinPeriodicCommands, self.spin_config_topic, self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Heard: "{msg}"')

        spin_commands = [
            value for cmd in msg.commands for value in (cmd.omega, cmd.duration)
        ]
        spin_period = msg.period

        self.get_logger().info(f'Sending: "{spin_commands}" and "{spin_period}"')
        self.send_request(spin_commands, spin_period)

    def send_request(self, spin_commands, spin_period):
        self.req.parameters = [
            Parameter(
                name=self.spin_commands_param_name, value=spin_commands
            ).to_parameter_msg(),
            Parameter(
                name=self.spin_period_param_name, value=spin_period
            ).to_parameter_msg(),
        ]
        self.future = self.cli.call_async(self.req)
        while not self.future.done():
            try:
                response = self.future.result()
            except Exception as e:
                self.get_logger().error("Service call failed with exception: %r" % (e,))
            break
        self.get_logger().info(f"Response: {response}")


def main(args=None):
    rclpy.init(args=args)

    topic_param_bridge = TopicParamBridge()

    rclpy.spin(topic_param_bridge)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    topic_param_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
