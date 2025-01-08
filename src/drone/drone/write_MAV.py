#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import pymavlink.dialects.v20.common as mavlink
from pymavlink import mavutil

class WriteMAV(Node):
    def __init__(self):
        super().__init__('write_MAV')
        
        self.subscription_1 = self.create_subscription(String, 'OUT/path', self.listener_callback_1, 10)
        self.subscription_1
        
        self.subscription_2 = self.create_subscription(String, 'IN/CAM/is_fire', self.listener_callback_1, 20)
        self.subscription_2

        self.subscription_3 = self.create_subscription(Bool, 'ps4_1/OUT/ps_button_state', self.listener_callback_3, 10)
        self.subscription_3
        self.subscription_4 = self.create_subscription(String, 'ps4_1/OUT/cmd_vel', self.listener_callback_1, 10)
        self.subscription_4

        # Initialize MAVLink connection
        self.mavlink_connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')

    def listener_callback_1(self, msg):
        self.get_logger().info(f"Received: '{msg.data}'")

    def listener_callback_3(self, ps_button_state):
        self.get_logger().info(f"Received: '{ps_button_state.data}'")
        if ps_button_state.data:
            print(type(ps_button_state.data))
            self.get_logger().info("PS Button is pressed => Arming drone")
            self.arm_drone()

    def arm_drone(self):
        # Create and send the arm command
        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        self.get_logger().info("Arm command sent")

def main(args=None):
    rclpy.init(args=args)
    node = WriteMAV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
