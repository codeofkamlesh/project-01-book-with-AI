"""
Isaac-ROS Bridge Example
This demonstrates integration between Isaac Sim and ROS2.
It shows how to connect Isaac's GPU-accelerated perception with ROS2.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import cv2
from cv_bridge import CvBridge


class IsaacROSBridge(Node):
    def __init__(self):
        super().__init__('isaac_ros_bridge')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Publishers for ROS2
        self.rgb_pub = self.create_publisher(Image, '/isaac/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/isaac/depth/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/isaac/camera_info', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/isaac/cmd_vel', 10)

        # Subscribers for ROS2
        self.command_sub = self.create_subscription(String, '/isaac/command', self.command_callback, 10)
        self.ros_cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.ros_command_callback, 10)

        # Timer for simulation loop
        self.timer = self.create_timer(0.1, self.simulation_loop)  # 10 Hz

        # Isaac Sim simulation state
        self.simulation_time = 0.0
        self.robot_position = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.robot_velocity = np.array([0.0, 0.0])  # linear, angular
        self.last_command = "idle"
        self.command_history = []

        self.get_logger().info('Isaac-ROS Bridge initialized')

    def command_callback(self, msg):
        """Receive high-level commands from ROS2."""
        command = msg.data
        self.last_command = command
        self.command_history.append((self.simulation_time, command))
        self.get_logger().info(f'Received Isaac command: {command}')

        # Process command and potentially send low-level commands
        self.process_high_level_command(command)

    def ros_command_callback(self, msg):
        """Receive low-level velocity commands from ROS2."""
        linear = msg.linear.x
        angular = msg.angular.z
        self.robot_velocity = np.array([linear, angular])
        self.get_logger().debug(f'Received velocity command: linear={linear:.2f}, angular={angular:.2f}')

    def process_high_level_command(self, command):
        """Process high-level commands and convert to low-level actions."""
        if "move forward" in command.lower():
            cmd = Twist()
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
        elif "turn left" in command.lower():
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
            self.cmd_vel_pub.publish(cmd)
        elif "turn right" in command.lower():
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = -0.5
            self.cmd_vel_pub.publish(cmd)
        elif "stop" in command.lower():
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)

    def simulation_loop(self):
        """Main simulation loop that simulates Isaac Sim behavior."""
        # Update simulation time
        self.simulation_time += 0.1  # 10 Hz loop

        # Update robot position based on velocity (simple kinematic model)
        dt = 0.1  # time step
        self.robot_position[0] += self.robot_velocity[0] * np.cos(self.robot_position[2]) * dt
        self.robot_position[1] += self.robot_velocity[0] * np.sin(self.robot_position[2]) * dt
        self.robot_position[2] += self.robot_velocity[1] * dt

        # Normalize angle
        self.robot_position[2] = ((self.robot_position[2] + np.pi) % (2 * np.pi)) - np.pi

        # Generate simulated sensor data
        self.publish_simulated_sensors()

        # Log robot state
        self.get_logger().debug(
            f'Sim time: {self.simulation_time:.1f}s, '
            f'Pos: ({self.robot_position[0]:.2f}, {self.robot_position[1]:.2f}, {self.robot_position[2]:.2f}), '
            f'Vel: ({self.robot_velocity[0]:.2f}, {self.robot_velocity[1]:.2f})'
        )

    def publish_simulated_sensors(self):
        """Publish simulated sensor data to ROS2."""
        # Create a simulated RGB image
        width, height = 640, 480
        rgb_image = np.zeros((height, width, 3), dtype=np.uint8)

        # Draw some simulated features
        center_x, center_y = width // 2, height // 2
        cv2.circle(rgb_image, (center_x, center_y), 50, (0, 255, 0), -1)  # Green circle
        cv2.rectangle(rgb_image, (100, 100), (200, 200), (255, 0, 0), 2)  # Blue square

        # Add position-dependent features
        obj_x = int(center_x + self.robot_position[0] * 50)
        obj_y = int(center_y + self.robot_position[1] * 50)
        if 0 <= obj_x < width and 0 <= obj_y < height:
            cv2.circle(rgb_image, (obj_x, obj_y), 30, (255, 255, 0), -1)  # Yellow object

        # Convert to ROS message and publish
        ros_image = self.cv_bridge.cv2_to_imgmsg(rgb_image, encoding="bgr8")
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = "camera_rgb_optical_frame"
        self.rgb_pub.publish(ros_image)

        # Create simulated depth image
        depth_image = np.ones((height, width), dtype=np.float32) * 5.0  # 5m default depth
        # Add depth variations based on robot position
        for y in range(height):
            for x in range(width):
                # Simulate closer objects in the center
                dist_from_center = np.sqrt((x - center_x)**2 + (y - center_y)**2)
                depth_image[y, x] = max(0.5, 5.0 - dist_from_center / 100.0)

        # Add robot-position dependent depth changes
        depth_offset = np.linalg.norm(self.robot_position[:2]) * 0.1
        depth_image = np.maximum(0.1, depth_image - depth_offset)

        # Convert to ROS message and publish
        ros_depth = self.cv_bridge.cv2_to_imgmsg(depth_image, encoding="32FC1")
        ros_depth.header.stamp = self.get_clock().now().to_msg()
        ros_depth.header.frame_id = "camera_depth_optical_frame"
        self.depth_pub.publish(ros_depth)

        # Publish camera info
        camera_info = CameraInfo()
        camera_info.header.stamp = self.get_clock().now().to_msg()
        camera_info.header.frame_id = "camera_rgb_optical_frame"
        camera_info.width = width
        camera_info.height = height
        camera_info.k = [500.0, 0.0, width/2, 0.0, 500.0, height/2, 0.0, 0.0, 1.0]  # Simple camera matrix
        self.camera_info_pub.publish(camera_info)

    def get_robot_state(self):
        """Get the current robot state."""
        return {
            'position': self.robot_position.copy(),
            'velocity': self.robot_velocity.copy(),
            'time': self.simulation_time,
            'last_command': self.last_command
        }


def main(args=None):
    rclpy.init(args=args)

    bridge = IsaacROSBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.get_logger().info('Shutting down Isaac-ROS Bridge')
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()