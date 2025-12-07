#!/usr/bin/env python3

"""
Add Two Ints Service Client example for ROS2.
This node calls the add_two_ints service.
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        self.request.a = a
        self.request.b = b
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)

    client = AddTwoIntsClient()

    if len(sys.argv) != 3:
        print('Usage: ros2 run examples_rclpy_services add_two_ints_client.py 1 2')
        return 1

    try:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    except ValueError:
        print('Arguments must be integers')
        return 1

    response = client.send_request(a, b)
    if response is not None:
        print(f'Result of {a} + {b} = {response.sum}')
    else:
        print('Service call failed')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()