#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool  # Import the Bool message

class PS4ControllerNode(Node):
    def __init__(self):
        super().__init__('ps4_controller1_node', namespace='ps4_1')  # Add namespace

        # Subscribe to the joystick topic
        self.sub1 = self.create_subscription(Joy, 'joy', self.joy_callback_1, 10)
        # Create a publisher for drone commands
        self.pub = self.create_publisher(Twist, 'OUT/cmd_vel', 10)
        # Create a publisher for the PS button state
        self.ps_pub = self.create_publisher(Bool, 'OUT/ps_button_state', 10)

        self.get_logger().info("PS4 Controller -1- Node is running...")

    def joy_callback_1(self, msg):
        # Log the buttons and axes values
        self.get_logger().info(f"Controller 1 - Buttons: {msg.buttons}")
        self.get_logger().info(f"Controller 1 - Axes: {msg.axes}")

        # Create a Twist message and set its values based on the joystick input
        twist = Twist()
        twist.linear.x = msg.axes[1]  # Forward/Backward
        twist.linear.y = msg.axes[0]  # Left/Right
        twist.linear.z = msg.axes[4]  # Up/Down
        twist.angular.z = msg.axes[3]  # Yaw

        # Publish the Twist message to control the drone
        self.pub.publish(twist)

        # Check the state of the PS button (assuming it's the 12th button)
        ps_button_state = Bool()
        ps_button_state.data = msg.buttons[10] == 1

        # Publish the PS button state
        self.ps_pub.publish(ps_button_state)

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)
    # Create an instance of the PS4ControllerNode
    node = PS4ControllerNode()
    try:
        # Spin the node to keep it running
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node and shutdown the ROS client library
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()