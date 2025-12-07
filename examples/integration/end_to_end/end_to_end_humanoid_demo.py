"""
End-to-End Humanoid Robot Control Demo
This example demonstrates integration of all four models:
1. ROS2 Foundations - Communication and control
2. Simulation - Environment and physics
3. Isaac - GPU-accelerated perception and control
4. VLA - Vision-Language-Action for natural interaction
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan, JointState
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import re


class EndToEndHumanoidDemo(Node):
    def __init__(self):
        super().__init__('end_to_end_humanoid_demo')

        # Initialize CV bridge
        self.cv_bridge = CvBridge()

        # Publishers for robot control
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)

        # Subscribers for sensor data
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        # Command subscriber for VLA-style commands
        self.command_sub = self.create_subscription(String, '/humanoid/command', self.command_callback, 10)

        # Timer for main control loop
        self.timer = self.create_timer(0.1, self.main_control_loop)  # 10 Hz

        # Robot state
        self.current_image = None
        self.current_scan = None
        self.current_odom = None
        self.current_joints = None
        self.command_queue = []
        self.robot_state = {
            'position': np.array([0.0, 0.0]),
            'orientation': 0.0,
            'velocity': np.array([0.0, 0.0]),
            'joints': {},
            'battery': 100.0,
            'status': 'idle'
        }

        # VLA system state
        self.vla_enabled = True
        self.command_history = []
        self.action_executing = False

        self.get_logger().info('End-to-End Humanoid Demo initialized')

    def image_callback(self, msg):
        """Process image data from simulation/Isaac."""
        try:
            self.current_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.robot_state['vision_processed'] = True
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def scan_callback(self, msg):
        """Process laser scan data."""
        self.current_scan = msg
        # Process scan for obstacles
        if len(msg.ranges) > 0:
            # Get min range to detect obstacles
            valid_ranges = [r for r in msg.ranges if r > 0 and not np.isinf(r) and not np.isnan(r)]
            if valid_ranges:
                min_range = min(valid_ranges)
                self.robot_state['obstacle_distance'] = min_range

    def odom_callback(self, msg):
        """Process odometry data."""
        self.current_odom = msg
        # Update robot position
        self.robot_state['position'][0] = msg.pose.pose.position.x
        self.robot_state['position'][1] = msg.pose.pose.position.y

        # Extract orientation from quaternion
        q = msg.pose.pose.orientation
        self.robot_state['orientation'] = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        # Extract linear velocity
        self.robot_state['velocity'][0] = msg.twist.twist.linear.x
        self.robot_state['velocity'][1] = msg.twist.twist.angular.z

    def joint_callback(self, msg):
        """Process joint state data."""
        self.current_joints = msg
        # Update joint positions
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.robot_state['joints'][name] = msg.position[i]

    def command_callback(self, msg):
        """Process high-level commands (VLA-style)."""
        command = msg.data
        self.command_queue.append(command)
        self.command_history.append({
            'time': self.get_clock().now().seconds_nanoseconds(),
            'command': command,
            'robot_state': self.robot_state.copy()
        })
        self.get_logger().info(f'Received VLA command: {command}')

    def main_control_loop(self):
        """Main control loop integrating all four models."""
        # Process any queued commands
        while self.command_queue and not self.action_executing:
            command = self.command_queue.pop(0)
            self.execute_vla_command(command)

        # Update robot status
        self.robot_state['status'] = 'executing' if self.action_executing else 'idle'

        # Log robot state periodically
        if int(self.get_clock().now().seconds_nanoseconds()[0]) % 5 == 0:
            self.get_logger().info(
                f'Robot state - Pos: ({self.robot_state["position"][0]:.2f}, {self.robot_state["position"][1]:.2f}), '
                f'Orientation: {self.robot_state["orientation"]:.2f}, '
                f'Battery: {self.robot_state["battery"]:.1f}%'
            )

    def execute_vla_command(self, command):
        """Execute a Vision-Language-Action command."""
        self.get_logger().info(f'Executing VLA command: {command}')
        self.action_executing = True

        # Step 1: Parse the command using VLA system
        parsed_action = self.parse_vla_command(command)

        if parsed_action:
            # Step 2: Plan action based on simulation environment
            action_plan = self.plan_action(parsed_action)

            # Step 3: Execute using Isaac's GPU-accelerated perception if available
            self.execute_action_with_perception(action_plan)

            # Step 4: Control through ROS2 communication
            self.execute_ros2_control(action_plan)

        self.action_executing = False

    def parse_vla_command(self, command):
        """Parse natural language command into structured action."""
        command_lower = command.lower().strip()

        # Define command patterns
        patterns = {
            'navigation': [
                r'(?:go to|move to|navigate to|go to location|reach) (.+)',
                r'(?:move|go) (.+)',
                r'(?:go|move) (?:forward|backward|left|right)'
            ],
            'manipulation': [
                r'(?:pick up|grasp|take|grab) (.+)',
                r'(?:place|put down|drop) (.+)',
                r'(?:lift|raise|move) (.+) (?:up|down|higher|lower)'
            ],
            'inspection': [
                r'(?:look at|examine|inspect|observe) (.+)',
                r'(?:face|turn toward|orient to) (.+)'
            ]
        }

        # Parse command
        for action_type, regex_list in patterns.items():
            for regex in regex_list:
                match = re.search(regex, command_lower)
                if match:
                    target = match.group(1) if len(match.groups()) > 0 else None
                    return {
                        'type': action_type,
                        'target': target,
                        'raw_command': command
                    }

        # If no specific pattern matched, return generic command
        return {
            'type': 'generic',
            'target': command,
            'raw_command': command
        }

    def plan_action(self, parsed_action):
        """Plan the action based on parsed command and current state."""
        action_plan = {
            'action_type': parsed_action['type'],
            'target': parsed_action['target'],
            'steps': [],
            'constraints': [],
            'success_criteria': []
        }

        if parsed_action['type'] == 'navigation':
            # Plan navigation sequence
            action_plan['steps'] = [
                {'step': 'localize', 'description': 'Localize robot in environment'},
                {'step': 'path_plan', 'description': 'Plan path to destination'},
                {'step': 'navigate', 'description': 'Execute navigation'},
                {'step': 'verify', 'description': 'Verify arrival at destination'}
            ]
            action_plan['success_criteria'] = ['reach_destination', 'no_collision']

        elif parsed_action['type'] == 'manipulation':
            # Plan manipulation sequence
            action_plan['steps'] = [
                {'step': 'locate_object', 'description': 'Locate target object'},
                {'step': 'approach_object', 'description': 'Approach object safely'},
                {'step': 'grasp_plan', 'description': 'Plan grasp motion'},
                {'step': 'execute_grasp', 'description': 'Execute grasp'},
                {'step': 'verify_grasp', 'description': 'Verify successful grasp'}
            ]
            action_plan['success_criteria'] = ['object_grasped', 'stable_grasp']

        elif parsed_action['type'] == 'inspection':
            # Plan inspection sequence
            action_plan['steps'] = [
                {'step': 'orient_to_target', 'description': 'Orient robot toward target'},
                {'step': 'capture_images', 'description': 'Capture images of target'},
                {'step': 'analyze', 'description': 'Analyze captured data'},
                {'step': 'report', 'description': 'Report findings'}
            ]
            action_plan['success_criteria'] = ['images_captured', 'analysis_completed']

        return action_plan

    def execute_action_with_perception(self, action_plan):
        """Execute action with Isaac's GPU-accelerated perception."""
        self.get_logger().info(f'Executing action with perception: {action_plan["action_type"]}')

        if self.current_image is not None:
            # Simulate perception processing using Isaac-style GPU acceleration
            self.get_logger().debug('Processing image with Isaac perception pipeline...')

            # In a real implementation, this would use Isaac's GPU-accelerated perception
            # For simulation, we'll just process the image
            height, width, _ = self.current_image.shape

            # Example: detect if there's a clear path ahead
            if action_plan['action_type'] == 'navigation':
                # Sample pixels in front of the robot
                center_x, center_y = width // 2, height // 2
                front_region = self.current_image[center_y:, center_x-50:center_x+50]

                # Calculate average brightness as a simple "obstacle detection"
                avg_brightness = np.mean(front_region)

                if avg_brightness < 50:  # Dark = potential obstacle
                    self.get_logger().warn('Potential obstacle detected ahead!')
                    action_plan['constraints'].append('avoid_obstacle')

    def execute_ros2_control(self, action_plan):
        """Execute action through ROS2 control interface."""
        self.get_logger().info(f'Executing ROS2 control for: {action_plan["action_type"]}')

        if action_plan['action_type'] == 'navigation':
            if 'target' in action_plan and action_plan['target']:
                # Simple navigation to a named location
                self.navigate_to_location(action_plan['target'])
            else:
                # Default navigation behavior
                self.move_forward(1.0)  # Move forward 1 meter

        elif action_plan['action_type'] == 'manipulation':
            if action_plan['target']:
                self.manipulate_object(action_plan['target'])

        elif action_plan['action_type'] == 'inspection':
            if action_plan['target']:
                self.inspect_target(action_plan['target'])

        else:
            # Stop robot for unrecognized commands
            self.stop_robot()

    def navigate_to_location(self, target_location):
        """Navigate to a specific location."""
        self.get_logger().info(f'Navigating to: {target_location}')

        # For demonstration, navigate to a relative position
        if "kitchen" in target_location.lower():
            self.navigate_to_coordinates(2.0, 1.5)
        elif "living room" in target_location.lower():
            self.navigate_to_coordinates(-1.0, 2.0)
        elif "bedroom" in target_location.lower():
            self.navigate_to_coordinates(0.0, -2.0)
        else:
            # Default navigation behavior
            self.move_forward(1.0)

    def navigate_to_coordinates(self, target_x, target_y):
        """Navigate to specific coordinates."""
        current_x = self.robot_state['position'][0]
        current_y = self.robot_state['position'][1]

        # Calculate distance and angle to target
        dx = target_x - current_x
        dy = target_y - current_y
        distance = np.sqrt(dx**2 + dy**2)

        if distance > 0.2:  # Threshold for reaching target
            # Calculate desired orientation
            desired_theta = np.arctan2(dy, dx)

            # Calculate orientation error
            angle_error = desired_theta - self.robot_state['orientation']
            # Normalize angle to [-pi, pi]
            while angle_error > np.pi:
                angle_error -= 2 * np.pi
            while angle_error < -np.pi:
                angle_error += 2 * np.pi

            # Create velocity command
            cmd = Twist()
            cmd.linear.x = min(0.5, distance * 0.5)  # Proportional control
            cmd.angular.z = max(-0.5, min(0.5, angle_error * 1.0))

            self.cmd_vel_pub.publish(cmd)
        else:
            # Stop when close to target
            self.stop_robot()

    def move_forward(self, distance):
        """Move robot forward by specified distance."""
        cmd = Twist()
        cmd.linear.x = 0.3  # Constant forward speed
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def manipulate_object(self, target_object):
        """Manipulate a target object."""
        self.get_logger().info(f'Manipulating object: {target_object}')

        # For simulation, just log the action
        # In a real implementation, this would involve:
        # 1. Approaching the object
        # 2. Identifying grasp points
        # 3. Executing grasp motion
        # 4. Verifying grasp success

    def inspect_target(self, target):
        """Inspect a target."""
        self.get_logger().info(f'Inspecting target: {target}')

        # Turn toward target for inspection
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.2  # Slow rotation for inspection
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        """Stop all robot motion."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def get_robot_status(self):
        """Get comprehensive robot status."""
        return {
            'position': self.robot_state['position'].tolist(),
            'orientation': self.robot_state['orientation'],
            'velocity': self.robot_state['velocity'].tolist(),
            'joints': self.robot_state['joints'],
            'battery': self.robot_state['battery'],
            'status': self.robot_state['status'],
            'command_queue_length': len(self.command_queue),
            'command_history_length': len(self.command_history)
        }


def main(args=None):
    rclpy.init(args=args)

    demo = EndToEndHumanoidDemo()

    try:
        rclpy.spin(demo)
    except KeyboardInterrupt:
        pass
    finally:
        demo.get_logger().info('Shutting down End-to-End Humanoid Demo')
        demo.stop_robot()  # Ensure robot stops
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()