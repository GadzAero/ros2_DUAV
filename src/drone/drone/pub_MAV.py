#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from std_msgs.msg import String
from std_msgs.msg import Float32


class MAVPublisher(Node):
    def __init__(self):
        super().__init__('pub_MAV')

        # Connexion à MAVLink
        self.connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762')  # Ajustez si nécessaire
        self.get_logger().info("En attente du heartbeat MAVLink...")
        self.connection.wait_heartbeat()
        self.get_logger().info("Connexion MAVLink établie !")


        self.publisher_1 = self.create_publisher(String, 'IN/MAV/gps_coord', 10)
        self.timer_1 = self.create_timer(1.0, self.timer_callback)  # 1 second interval
        self.get_logger().info("Publisher Node Started")
        
        self.publisher_2 = self.create_publisher(String, 'IN/MAV/fences', 10)
        self.timer_2 = self.create_timer(1.0, self.timer_callback)  # 1 second interval
        self.get_logger().info("Publisher Node Started")
        
        self.publisher_3 = self.create_publisher(String, 'IN/MAV/altitude', 10)
        self.timer_3 = self.create_timer(1.0, self.timer_callback)  # 1 second interval
        self.get_logger().info("Publisher Node Started")
        
        self.publisher_4 = self.create_publisher(String, 'IN/MAV/no_go_zones', 10)
        self.timer_4 = self.create_timer(1.0, self.timer_callback)  # 1 second interval
        self.get_logger().info("Publisher Node Started")
        
        self.publisher_5 = self.create_publisher(String, 'IN/MAV/imu', 10)
        self.timer_5 = self.create_timer(1.0, self.timer_callback)  # 1 second interval
        self.get_logger().info("Publisher Node Started")
        
        self.publisher_6 = self.create_publisher(String, 'IN/MAV/attitude', 10)
        self.timer_6 = self.create_timer(1.0, self.timer_callback)  # 1 second interval
        self.get_logger().info("Publisher Node Started")

        self.publisher_battery = self.create_publisher(Float32, 'IN/MAV/battery', 10)
        self.timer_7 = self.create_timer(1.0, self.timer_callback_battery)  # 1 second interval
        self.get_logger().info("Pub Node Started : BATTERY")


    def timer_callback(self):
        msg = String()
        msg.data = "MAV Status: Active"
        self.publisher_1.publish(msg)
        self.get_logger().info(f"Publishing: '{msg.data}'")


    def timer_callback_battery(self):
        try:
            msg = self.connection.recv_match(type='BATTERY_STATUS', blocking=True)
            if msg:
                voltage_msg = Float32()
                voltage_msg.data = msg.voltages[0] / 1000.0  # En volts
                self.publisher_battery.publish(voltage_msg)
                self.get_logger().info("OK --> Battery")
            else:
                self.get_logger().warning("Aucun message BATTERY_STATUS reçu.")
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la récupération des données MAVLink: {e}")





def main(args=None):
    rclpy.init(args=args)
    node = MAVPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
