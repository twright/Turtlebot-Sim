import rclpy
from rclpy.node import Node

from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters


class MinimalClientAsync(Node):

    def __init__(
        self,
        service_name="/controller_server/set_parameters",
        spin_period_param_name="FollowPath.spin_period",
        spin_commands_param_name="FollowPath.spin_commands",
    ):
        super().__init__("topic_param_bridge")
        self.service_name = service_name
        self.spin_period_param_name = spin_period_param_name
        self.spin_commands_param_name = spin_commands_param_name

        self.cli = self.create_client(SetParameters, self.service_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = SetParameters.Request()

    def send_request(self):
        self.req.parameters = [
            Parameter(name=self.spin_period_param_name, value=5.0).to_parameter_msg()
        ]
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info("Service call failed %r" % (e,))
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
