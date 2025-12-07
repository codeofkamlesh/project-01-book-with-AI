"""
Prompt-to-Action system for VLA (Vision-Language-Action) pipeline.
This example demonstrates how to convert natural language prompts into robot actions.
"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import re


class PromptToAction:
    def __init__(self):
        """Initialize the prompt-to-action system."""
        self.cv_bridge = CvBridge()
        self.current_image = None
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

        # ROS publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.prompt_sub = rospy.Subscriber('/vla/prompt', String, self.prompt_callback)

        print("Prompt-to-Action system initialized!")

    def image_callback(self, msg: Image):
        """Callback for processing incoming camera images."""
        try:
            self.current_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print(f"Error converting image: {e}")

    def prompt_callback(self, msg: String):
        """Callback for processing incoming prompts."""
        prompt = msg.data
        print(f"Received prompt: {prompt}")
        self.process_prompt(prompt)

    def process_prompt(self, prompt: str):
        """Process the natural language prompt and execute action."""
        # Parse the prompt to extract action and target
        action, target = self.parse_prompt(prompt)

        if action and target:
            print(f"Parsed action: {action}, target: {target}")
            self.execute_action(action, target)
        else:
            print("Could not parse prompt")
            self.execute_default_action()

    def parse_prompt(self, prompt: str) -> tuple:
        """Parse the prompt to extract action and target."""
        prompt_lower = prompt.lower().strip()

        # Define action patterns
        action_patterns = {
            'pick': [r'pick up (.+)', r'grasp (.+)', r'grab (.+)', r'take (.+)'],
            'place': [r'place (.+)', r'put (.+)', r'drop (.+)'],
            'move': [r'move to (.+)', r'go to (.+)', r'navigate to (.+)', r'approach (.+)'],
            'follow': [r'follow (.+)', r'go after (.+)'],
            'inspect': [r'look at (.+)', r'inspect (.+)', r'examine (.+)']
        }

        # Look for actions in the prompt
        for action, patterns in action_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, prompt_lower)
                if match:
                    target = match.group(1).strip()
                    return action, target

        # If no specific target, try to extract any noun phrase
        if 'pick' in prompt_lower or 'grasp' in prompt_lower or 'take' in prompt_lower:
            return 'pick', 'nearest object'
        elif 'move' in prompt_lower or 'go' in prompt_lower or 'navigate' in prompt_lower:
            return 'move', 'specified location'
        elif 'follow' in prompt_lower:
            return 'follow', 'target'
        elif 'look' in prompt_lower or 'inspect' in prompt_lower:
            return 'inspect', 'target'
        else:
            return None, None

    def execute_action(self, action: str, target: str):
        """Execute the parsed action."""
        if action == 'pick':
            self.execute_pick_action(target)
        elif action == 'place':
            self.execute_place_action(target)
        elif action == 'move':
            self.execute_move_action(target)
        elif action == 'follow':
            self.execute_follow_action(target)
        elif action == 'inspect':
            self.execute_inspect_action(target)
        else:
            print(f"Unknown action: {action}")
            self.execute_default_action()

    def execute_pick_action(self, target: str):
        """Execute a pick action."""
        print(f"Attempting to pick up: {target}")
        # In a real implementation, this would:
        # 1. Navigate to the object
        # 2. Align with the object
        # 3. Lower the gripper
        # 4. Close the gripper
        # 5. Lift the object
        self.navigate_to_object(target)

    def execute_place_action(self, target: str):
        """Execute a place action."""
        print(f"Attempting to place object at: {target}")
        # In a real implementation, this would:
        # 1. Navigate to the placement location
        # 2. Lower the object
        # 3. Open the gripper
        self.navigate_to_location(target)

    def execute_move_action(self, target: str):
        """Execute a move action."""
        print(f"Moving to: {target}")
        # Navigate to the target location
        self.navigate_to_location(target)

    def execute_follow_action(self, target: str):
        """Execute a follow action."""
        print(f"Following: {target}")
        # In a real implementation, this would:
        # 1. Track the target
        # 2. Maintain a safe distance
        # 3. Follow the target's movement
        self.follow_target(target)

    def execute_inspect_action(self, target: str):
        """Execute an inspect action."""
        print(f"Inspecting: {target}")
        # Navigate close to the target for inspection
        self.navigate_close_to_target(target)

    def execute_default_action(self):
        """Execute a default action when parsing fails."""
        print("Executing default action: stop")
        self.stop_robot()

    def navigate_to_object(self, target: str):
        """Navigate to a specific object."""
        # In a real implementation, this would use object detection
        # to find the object's location and navigate to it
        print(f"Navigating to {target}")
        # For demonstration, we'll move forward a bit
        self.move_forward(1.0)

    def navigate_to_location(self, target: str):
        """Navigate to a specific location."""
        # In a real implementation, this would interpret the location
        # and navigate to the corresponding coordinates
        print(f"Navigating to {target}")
        # For demonstration, we'll rotate and move forward
        self.rotate(0.5)
        self.move_forward(1.0)

    def follow_target(self, target: str):
        """Follow a target."""
        # In a real implementation, this would continuously
        # track the target and adjust position accordingly
        print(f"Following {target}")
        self.move_forward(0.5)

    def navigate_close_to_target(self, target: str):
        """Navigate close to a target for inspection."""
        print(f"Navigating close to {target}")
        self.move_forward(0.8)

    def move_forward(self, distance: float):
        """Move the robot forward by a specified distance."""
        # Simple open-loop movement for demonstration
        duration = distance / 0.5  # Assuming 0.5 m/s speed
        cmd = Twist()
        cmd.linear.x = 0.5  # Move forward at 0.5 m/s

        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        # Stop the robot
        self.stop_robot()

    def rotate(self, angle: float):
        """Rotate the robot by a specified angle."""
        # Simple open-loop rotation for demonstration
        duration = abs(angle) / 0.5  # Assuming 0.5 rad/s rotation speed
        cmd = Twist()
        cmd.angular.z = 0.5 if angle > 0 else -0.5  # Rotate in the specified direction

        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now()

        while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        # Stop the robot
        self.stop_robot()

    def stop_robot(self):
        """Stop the robot's movement."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        print("Robot stopped")


def main():
    """Main function to run the prompt-to-action system."""
    rospy.init_node('prompt_to_action', anonymous=True)

    try:
        prompt_to_action = PromptToAction()
        print("Prompt-to-Action system running. Waiting for prompts...")

        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        print("ROS Interrupt received")
    except Exception as e:
        print(f"Error running prompt-to-action system: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()