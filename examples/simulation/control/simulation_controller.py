#!/usr/bin/env python3

"""
Simulation controller for the simple robot model.
This node controls the robot in the simulation environment.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class SimulationController(Node):
    def __init__(self):
        super().__init__('simulation_controller')

        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/simple_robot/cmd_vel', 10)

        # Create subscriber for odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/simple_robot/odom',
            self.odom_callback,
            10
        )

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        # Variables for robot state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.target_x = 2.0
        self.target_y = 2.0

        self.get_logger().info('Simulation controller initialized')

    def odom_callback(self, msg):
        """Update robot pose from odometry message."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def control_loop(self):
        """Main control loop."""
        # Calculate distance to target
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx*dx + dy*dy)

        # Calculate angle to target
        target_angle = math.atan2(dy, dx)

        # Calculate angular error
        angle_error = target_angle - self.current_yaw
        # Normalize angle error to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # Create twist message
        cmd_msg = Twist()

        # Simple proportional controller
        if distance > 0.1:  # If not close to target
            cmd_msg.linear.x = min(0.5, distance * 0.5)  # Forward speed
            cmd_msg.angular.z = min(0.5, max(-0.5, angle_error * 1.0))  # Angular speed
        else:
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0

        # Publish command
        self.cmd_vel_pub.publish(cmd_msg)

        # Log position
        self.get_logger().info(f'Pos: ({self.current_x:.2f}, {self.current_y:.2f}), '
                              f'Dist to target: {distance:.2f}')


def main(args=None):
    rclpy.init(args=args)

    controller = SimulationController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()