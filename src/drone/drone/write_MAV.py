#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from mavros_msgs.srv import CommandBool

class WriteMAV(Node):
    def __init__(self):
        super().__init__('write_MAV')

        # Creating client for services
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/cmd/arming service...')
        self.get_logger().info('Connected to /mavros/cmd/arming service.')
        
        # From autonomous 
        self.subscription_1 = self.create_subscription(String, 'OUT/path', self.lc_1, 10)
        self.subscription_2 = self.create_subscription(String, 'IN/CAM/is_fire', self.lc_1, 20)

        # From PS4_1
        self.subscription_4 = self.create_subscription(Twist, 'ps4_1/OUT/cmd_vel', self.lc_cmd_vel, 10)
        self.subscription_arm = self.create_subscription(Bool, 'ps4_1/OUT/share_button_state', self.lc_arm, 10)
        self.subscription_disarm = self.create_subscription(Bool, 'ps4_1/OUT/options_button_state', self.lc_disarm, 10)

        self.get_logger().info('Write_MAV.__init__ OK !')

    def lc_1(self, msg):   # lc : listener_callback_...
        self.get_logger().debug(f"Received: '{msg.data}'")
        
    def lc_arm(self, share_button_state):
        self.get_logger().debug(f"Received Share button state: '{share_button_state.data}'")
        if share_button_state.data:
            self.arm_drone()

    def lc_disarm(self, options_button_state):
        self.get_logger().debug(f"Received Options button state: '{options_button_state.data}'")
        if options_button_state.data:
            self.get_logger().info("DISARMING")
            self.disarm_drone()

    def lc_cmd_vel(self, msg):
        self.get_logger().debug(f"Received cmd_vel: linear=({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), angular=({msg.angular.x}, {msg.angular.y}, {msg.angular.z})")
        self.send_manual_control_command(msg)

    def arm_drone(self):
        request = CommandBool.Request()
        request.value = True
        future = self.arm_client.call_async(request)
        self.get_logger().info("Sent arm request, waiting for response...")
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            result = future.result()
            if result.success:
                self.get_logger().info("Drone armed successfully!")
            else:
                self.get_logger().error("Failed to arm the drone.")
        else:
            self.get_logger().error("Future not completed. Timeout?")

    def disarm_drone(self):
        self.get_logger().info("Drone en cours de désarmement...")
        if not self.arm_client.service_is_ready():
            self.get_logger().error("Service /mavros/cmd/disarming non disponible")
            return
        request = CommandBool.Request()
        request.value = False
        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("ok")
        try:
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info("Drone désarmé avec succès")
                else:
                    self.get_logger().error("Le service a répondu, mais le désarmement a échoué")
            else:
                self.get_logger().error("Le service a répondu, mais aucune donnée valide n'a été reçue")
        except Exception as e:
            self.get_logger().error(f"Erreur lors de l'appel au service: {e}")

    def send_manual_control_command(self, twist):
        pass

    # def send_manual_control_command(self, twist):
    #     # Convert the Twist message to MANUAL_CONTROL values
    #     x = int(twist.linear.x * 1000)  # Forward/Backward
    #     y = int(twist.linear.y * 1000)  # Left/Right
    #     z = int((twist.linear.z + 1) * 500)  # Throttle (0-1000)
    #     r = int(twist.angular.z * 1000)  # Yaw

    #     # Send the MANUAL_CONTROL message
    #     self.mavlink_connection.mav.manual_control_send(
    #         self.mavlink_connection.target_system,
    #         x,
    #         0, # y,
    #         0, # z,
    #         0, # r,
    #         0  # No buttons pressed
    #     )
    #     self.get_logger().info(f"Manual control command sent: x={x}, y={y}, z={z}, r={r}")

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

    # def disarm_drone(self):
    #     # Create and send the disarm command
    #     self.mavlink_connection.mav.command_long_send(
    #         self.mavlink_connection.target_system,
    #         self.mavlink_connection.target_component,
    #         mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    #         0,
    #         0, 0, 0, 0, 0, 0, 0
    #     )
    #     self.get_logger().info("Disarm command sent")

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
