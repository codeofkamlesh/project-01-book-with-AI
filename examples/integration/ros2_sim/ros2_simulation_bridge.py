#!/usr/bin/env python3

"""
ROS2-Simulation Bridge Example
This node demonstrates integration between ROS2 and simulation environments.
It publishes commands to simulation and subscribes to simulated sensor data.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan, Image, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np


class ROS2SimulationBridge(Node):
    def __init__(self):
        super().__init__('ros2_simulation_bridge')

        # Publishers for sending commands to simulation
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)

        # Subscribers for receiving data from simulation
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        # Robot state variables
        self.current_odom = None
        self.current_scan = None
        self.current_image = None
        self.current_joints = None
        self.command_queue = []

        # Goal position for navigation
        self.goal_x = 2.0
        self.goal_y = 2.0

        self.get_logger().info('ROS2-Simulation Bridge initialized')

    def odom_callback(self, msg):
        """Receive odometry data from simulation."""
        self.current_odom = msg
        self.get_logger().debug(f'Received odometry: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}')

    def scan_callback(self, msg):
        """Receive laser scan data from simulation."""
        self.current_scan = msg
        # Log the minimum range for debugging
        if len(msg.ranges) > 0:
            min_range = min([r for r in msg.ranges if r > 0 and not np.isinf(r)], default=float('inf'))
            self.get_logger().debug(f'Laser scan: min range = {min_range:.2f}m')

    def image_callback(self, msg):
        """Receive image data from simulation."""
        self.current_image = msg
        # Just log that we received an image
        self.get_logger().debug(f'Received image: {msg.width}x{msg.height}')

    def joint_state_callback(self, msg):
        """Receive joint state data from simulation."""
        self.current_joints = msg
        if len(msg.position) > 0:
            self.get_logger().debug(f'Received joint states: {len(msg.position)} joints')

    def control_loop(self):
        """Main control loop that integrates ROS2 with simulation."""
        if self.current_odom is not None:
            # Get current position
            current_x = self.current_odom.pose.pose.position.x
            current_y = self.current_odom.pose.pose.position.y

            # Calculate distance to goal
            dx = self.goal_x - current_x
            dy = self.goal_y - current_y
            distance_to_goal = (dx**2 + dy**2)**0.5

            # Simple proportional controller
            cmd = Twist()

            if distance_to_goal > 0.2:  # If not close to goal
                # Calculate desired direction
                angle_to_goal = np.arctan2(dy, dx)

                # Get current orientation from quaternion
                q = self.current_odom.pose.pose.orientation
                current_yaw = np.arctan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                )

                # Calculate angular error
                angle_error = angle_to_goal - current_yaw
                # Normalize angle to [-pi, pi]
                while angle_error > np.pi:
                    angle_error -= 2 * np.pi
                while angle_error < -np.pi:
                    angle_error += 2 * np.pi

                # Set velocities
                cmd.linear.x = min(0.5, distance_to_goal * 0.5)  # Forward speed
                cmd.angular.z = max(-0.5, min(0.5, angle_error * 1.0))  # Angular speed
            else:
                # Stop when close to goal
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0

            # Publish command to simulation
            self.cmd_vel_pub.publish(cmd)

            # Log position and command
            self.get_logger().info(
                f'Pos: ({current_x:.2f}, {current_y:.2f}), '
                f'Dist to goal: {distance_to_goal:.2f}, '
                f'Cmd: lin={cmd.linear.x:.2f}, ang={cmd.angular.z:.2f}'
            )

    def send_joint_commands(self, joint_positions):
        """Send joint commands to the simulated robot."""
        joint_cmd = JointState()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        joint_cmd.name = [f'joint_{i}' for i in range(len(joint_positions))]
        joint_cmd.position = joint_positions
        self.joint_cmd_pub.publish(joint_cmd)


def main(args=None):
    rclpy.init(args=args)

    bridge = ROS2SimulationBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()