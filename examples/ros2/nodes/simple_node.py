#!/usr/bin/env python3

"""
Simple ROS2 node example.
This demonstrates the basic structure of a ROS2 node.
"""

import rclpy
from rclpy.node import Node


class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')

        # Log a message when the node is initialized
        self.get_logger().info('Simple node has been started')

        # Declare a parameter
        self.declare_parameter('example_param', 'default_value')

        # Get the parameter value
        param_value = self.get_parameter('example_param').value
        self.get_logger().info(f'Parameter value: {param_value}')

        # Create a timer
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        self.get_logger().info(f'Timer callback called {self.counter} times')
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)

    node = SimpleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()