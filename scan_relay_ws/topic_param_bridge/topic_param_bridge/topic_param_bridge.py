from collections import namedtuple
import subprocess

import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.msg import ParameterType
from ros2cli.node.direct import DirectNode
from rcl_interfaces.srv import SetParameters
from rclpy.node import HIDDEN_NODE_PREFIX
from ros2cli.node.strategy import NodeStrategy

from spin_interfaces.msg import SpinPeriodicCommands


def call_set_parameters(*, node, node_name, parameters):
    # create client
    client = node.create_client(SetParameters, f"{node_name}/set_parameters")

    # call as soon as ready
    ready = client.wait_for_service(timeout_sec=5.0)
    if not ready:
        raise RuntimeError("Wait for service timed out")

    request = SetParameters.Request()
    request.parameters = parameters
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    # handle response
    response = future.result()
    return response


NodeName = namedtuple("NodeName", ("name", "namespace", "full_name"))


def get_node_names(*, node, include_hidden_nodes=False):
    node_names_and_namespaces = node.get_node_names_and_namespaces()
    return [
        NodeName(
            name=t[0],
            namespace=t[1],
            full_name=t[1] + ("" if t[1].endswith("/") else "/") + t[0],
        )
        for t in node_names_and_namespaces
        if (include_hidden_nodes or (t[0] and not t[0].startswith(HIDDEN_NODE_PREFIX)))
    ]


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
        cmd_str = f"""ros2 param set /controller_server FollowPath.spin_period {spin_period} """
        subprocess.run(cmd_str, shell=True)

        """Note: Attempt at doing it "the right way" but it doesn't work. The error is due to DirectNode and not having `arg`"""
        # node_name = "/controller_server"
        # commands_parameter = Parameter()
        # period_parameter = Parameter()
        # period_parameter.name = "spin_period"
        # period_parameter_value = ParameterValue()
        # period_parameter_value.type = ParameterType.PARAMETER_DOUBLE
        # period_parameter_value.double_value = spin_period
        # period_parameter.value = period_parameter_value

        # commands_parameter_value = ParameterValue()
        # commands_parameter_value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        # commands_parameter_value._double_array_value = spin_commands
        # commands_parameter.name = "spin_commands"
        # commands_parameter.value = commands_parameter_value

        # with NodeStrategy([]) as node:
        #     node_names = get_node_names(node=node, include_hidden_nodes=False)

        # if node_name not in {n.full_name for n in node_names}:
        #     self.get_logger().error("Node not found")
        #     return

        # args_mock = type("", (object,), {"argv": 0})()
        # with DirectNode(args_mock) as node:
        #     response = call_set_parameters(
        #         node=node,
        #         node_name=node_name,
        #         parameters=[commands_parameter, period_parameter],
        #     )

        #     # output response
        #     for result in response.results:
        #         if result.successful:
        #             msg = "Set parameter successful"
        #             if result.reason:
        #                 msg += ": " + result.reason
        #             self.get_logger().info(msg)
        #         else:
        #             msg = "Setting parameter failed"
        #             if result.reason:
        #                 msg += ": " + result.reason
        #             self.get_logger().error(msg)


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
