#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from mavros_msgs.srv import CommandBool
import pymavlink.dialects.v20.common as mavlink
from pymavlink import mavutil

class WriteMAV(Node):
    def __init__(self):
        super().__init__('write_MAV')

        # Connexion à MAVLink
        try:
            self.mavlink_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762') 
            # self.mavlink_connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
            # self.mavlink_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
            self.get_logger().info("En attente du heartbeat MAVLink...")
        except:
            self.get_logger().error("Impossible de se connecter à MAVLink")
        # finally:
            return
        self.mavlink_connection.wait_heartbeat()
        self.get_logger().info("Connexion MAVLink établie (system %u component %u) !" % (self.mavlink_connection.target_system, self.mavlink_connection.target_system))

        
        # From autonomous 
        self.subscription_1 = self.create_subscription(String, 'OUT/path', self.listener_callback_1, 10)
        self.subscription_2 = self.create_subscription(String, 'IN/CAM/is_fire', self.listener_callback_1, 20)

        # From PS4_1
        self.subscription_3 = self.create_subscription(Bool, 'ps4_1/OUT/ps_button_state', self.listener_callback_3, 10)
        self.subscription_4 = self.create_subscription(Twist, 'ps4_1/OUT/cmd_vel', self.listener_callback_cmd_vel, 10)
        self.subscription_5 = self.create_subscription(Bool, 'ps4_1/OUT/share_button_state', self.listener_callback_5, 10)
        self.subscription_6 = self.create_subscription(Bool, 'ps4_1/OUT/options_button_state', self.listener_callback_6, 10)

    def listener_callback_1(self, msg):
        self.get_logger().debug(f"Received: '{msg.data}'")

    def listener_callback_3(self, ps_button_state):
        self.get_logger().debug(f"Received: '{ps_button_state.data}'")
        
    def listener_callback_5(self, share_button_state):
        self.get_logger().debug(f"Received Share button state: '{share_button_state.data}'")
        if share_button_state.data:
            self.get_logger().info("ARMING")
            self.arm_drone()

    def listener_callback_6(self, options_button_state):
        self.get_logger().debug(f"Received Options button state: '{options_button_state.data}'")
        if options_button_state.data:
            self.get_logger().info("DISARMING")
            self.disarm_drone()

    def listener_callback_cmd_vel(self, msg):
        self.get_logger().debug(f"Received cmd_vel: linear=({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), angular=({msg.angular.x}, {msg.angular.y}, {msg.angular.z})")
        self.send_manual_control_command(msg)

    def send_manual_control_command(self, twist):
        # Convert the Twist message to MANUAL_CONTROL values
        x = int(twist.linear.x * 1000)  # Forward/Backward
        y = int(twist.linear.y * 1000)  # Left/Right
        z = int((twist.linear.z + 1) * 500)  # Throttle (0-1000)
        r = int(twist.angular.z * 1000)  # Yaw

        # Send the MANUAL_CONTROL message
        self.mavlink_connection.mav.manual_control_send(
            self.mavlink_connection.target_system,
            x,
            0, # y,
            0, # z,
            0, # r,
            0  # No buttons pressed
        )
        self.get_logger().info(f"Manual control command sent: x={x}, y={y}, z={z}, r={r}")

    # def arm_drone(self):
    #     # Create and send the arm command
    #     self.mavlink_connection.mav.command_long_send(
    #         self.mavlink_connection.target_system,
    #         self.mavlink_connection.target_component,
    #         mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    #         0,
    #         1, 0, 0, 0, 0, 0, 0
    #     )
    #     self.get_logger().info("Arm command sent")

    def arm_drone(self):
        request = CommandBool.Request()
        request.value = True  # Set to False to disarm
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Arming result: {future.result().success}")
        else:
            self.get_logger().error("Failed to call arming service")

    def disarm_drone(self):
        # Create and send the disarm command
        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        self.get_logger().info("Disarm command sent")

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
