"""
Simple humanoid control example using VLA (Vision-Language-Action) system.
This demonstrates basic humanoid robot control based on natural language commands.
"""

import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
from collections import deque
import numpy as np


class SimpleHumanoidController:
    def __init__(self):
        """Initialize the humanoid controller."""
        self.cv_bridge = CvBridge()
        self.command_history = deque(maxlen=10)  # Keep last 10 commands
        self.current_image = None
        self.current_joints = None
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.tf_listener = tf.TransformListener()

        # ROS publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.joint_cmd_pub = rospy.Publisher('/joint_commands', JointState, queue_size=10)
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        self.command_sub = rospy.Subscriber('/vla/command', String, self.command_callback)

        print("Simple Humanoid Controller initialized!")

    def image_callback(self, msg: Image):
        """Callback for processing incoming camera images."""
        try:
            self.current_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print(f"Error converting image: {e}")

    def joint_callback(self, msg: JointState):
        """Callback for processing incoming joint states."""
        self.current_joints = msg

    def command_callback(self, msg: String):
        """Callback for processing incoming commands."""
        command = msg.data
        self.command_history.append(command)
        print(f"Received command: {command}")
        self.process_command(command)

    def process_command(self, command: str):
        """Process the natural language command and execute action."""
        command_lower = command.lower()

        # Simple command parsing
        if "walk" in command_lower or "move forward" in command_lower:
            self.walk_forward()
        elif "turn left" in command_lower:
            self.turn_left()
        elif "turn right" in command_lower:
            self.turn_right()
        elif "stop" in command_lower or "halt" in command_lower:
            self.stop_robot()
        elif "raise" in command_lower and "arm" in command_lower:
            self.raise_arm()
        elif "lower" in command_lower and "arm" in command_lower:
            self.lower_arm()
        elif "wave" in command_lower:
            self.wave_arm()
        elif "dance" in command_lower:
            self.perform_dance()
        elif "look" in command_lower or "face" in command_lower:
            self.look_at_command(command)
        else:
            print(f"Unknown command: {command}")
            self.execute_default_behavior()

    def walk_forward(self):
        """Make the robot walk forward."""
        print("Walking forward...")
        cmd = Twist()
        cmd.linear.x = 0.3  # Forward speed
        cmd.angular.z = 0.0  # No rotation
        self.cmd_vel_pub.publish(cmd)

    def turn_left(self):
        """Turn the robot left."""
        print("Turning left...")
        cmd = Twist()
        cmd.linear.x = 0.0  # No forward movement
        cmd.angular.z = 0.5  # Left rotation
        self.cmd_vel_pub.publish(cmd)

    def turn_right(self):
        """Turn the robot right."""
        print("Turning right...")
        cmd = Twist()
        cmd.linear.x = 0.0  # No forward movement
        cmd.angular.z = -0.5  # Right rotation
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        """Stop the robot's movement."""
        print("Stopping robot...")
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def raise_arm(self):
        """Raise the robot's arm."""
        print("Raising arm...")
        if self.current_joints:
            joint_cmd = JointState()
            joint_cmd.name = self.current_joints.name.copy()
            joint_cmd.position = list(self.current_joints.position)  # Make mutable

            # Modify joint positions to raise arm
            # This is a simplified example - in reality, you'd target specific joints
            for i, name in enumerate(joint_cmd.name):
                if "shoulder" in name or "elbow" in name:
                    # Increase joint angle to raise arm
                    joint_cmd.position[i] += 0.2  # Small increment

            joint_cmd.header.stamp = rospy.Time.now()
            self.joint_cmd_pub.publish(joint_cmd)

    def lower_arm(self):
        """Lower the robot's arm."""
        print("Lowering arm...")
        if self.current_joints:
            joint_cmd = JointState()
            joint_cmd.name = self.current_joints.name.copy()
            joint_cmd.position = list(self.current_joints.position)  # Make mutable

            # Modify joint positions to lower arm
            for i, name in enumerate(joint_cmd.name):
                if "shoulder" in name or "elbow" in name:
                    # Decrease joint angle to lower arm
                    joint_cmd.position[i] -= 0.2  # Small decrement

            joint_cmd.header.stamp = rospy.Time.now()
            self.joint_cmd_pub.publish(joint_cmd)

    def wave_arm(self):
        """Make the robot wave its arm."""
        print("Waving arm...")
        # This would involve sending a sequence of joint commands
        # to create a waving motion
        if self.current_joints:
            # Wave by alternating shoulder and elbow angles
            joint_cmd = JointState()
            joint_cmd.name = self.current_joints.name.copy()
            joint_cmd.position = list(self.current_joints.position)  # Make mutable

            for i, name in enumerate(joint_cmd.name):
                if "shoulder" in name:
                    # Add oscillation to shoulder joint
                    joint_cmd.position[i] += 0.3 * np.sin(rospy.Time.now().to_sec())
                elif "elbow" in name:
                    # Add oscillation to elbow joint
                    joint_cmd.position[i] += 0.2 * np.cos(rospy.Time.now().to_sec())

            joint_cmd.header.stamp = rospy.Time.now()
            self.joint_cmd_pub.publish(joint_cmd)

    def perform_dance(self):
        """Make the robot perform a simple dance."""
        print("Performing dance...")
        # This would involve a sequence of coordinated movements
        # For simplicity, we'll alternate between walking and turning
        cmd = Twist()
        cmd.linear.x = 0.2 * np.sin(rospy.Time.now().to_sec())
        cmd.angular.z = 0.3 * np.cos(rospy.Time.now().to_sec())
        self.cmd_vel_pub.publish(cmd)

    def look_at_command(self, command: str):
        """Parse look at command and orient robot."""
        print(f"Looking at command: {command}")
        # Simple parsing to look in a direction
        if "left" in command:
            self.turn_left()
        elif "right" in command:
            self.turn_right()
        elif "front" in command or "forward" in command:
            self.stop_robot()
        else:
            # Default to looking forward
            self.stop_robot()

    def execute_default_behavior(self):
        """Execute a default behavior when command is not understood."""
        print("Executing default behavior: stop and beep")
        self.stop_robot()
        # In a real robot, this might trigger an audio confirmation
        # that the command wasn't understood

    def run(self):
        """Main run loop."""
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            # Continue any ongoing behaviors
            rate.sleep()


def main():
    """Main function to run the humanoid controller."""
    rospy.init_node('simple_humanoid_controller', anonymous=True)

    try:
        controller = SimpleHumanoidController()
        print("Simple Humanoid Controller running. Waiting for commands...")

        # Keep the node running
        controller.run()
    except rospy.ROSInterruptException:
        print("ROS Interrupt received")
    except Exception as e:
        print(f"Error running humanoid controller: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()