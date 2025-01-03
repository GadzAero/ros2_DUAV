#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class PS4ControllerNode(Node):
    def __init__(self):
        super().__init__('ps4_controller1_node')

        self.sub1 = self.create_subscription(Joy, '/controller_1/joy', self.joy_callback_1, 10)

        self.get_logger().info("PS4 Controller -1- Node is running...")

    def joy_callback_1(self, msg):
        self.get_logger().info(f"Controller 1 - Buttons: {msg.buttons}")
        self.get_logger().info(f"Controller 1 - Axes: {msg.axes}")

def main(args=None):
    rclpy.init(args=args)
    node = PS4ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()