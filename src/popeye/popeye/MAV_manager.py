#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
import pymavlink.dialects.v20.common as mavlink
from pymavlink import mavutil

class MAVManager(Node):
    def __init__(self):
        super().__init__('MAV_manager', namespace='POPEYE')

        # Connexion à MAVLink
        try:
            # self.mavlink_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5760', baud=115200)
            self.mavlink_connection = mavutil.mavlink_connection('/dev/ttyACM1', baud=115200)
        except Exception as e:
            self.get_logger().error(f"Erreur MAVLink : {e}")
            raise RuntimeError("Impossible de se connecter à MAVLink.")
        self.get_logger().info("En attente du heartbeat MAVLink...")
        self.mavlink_connection.wait_heartbeat()
        self.get_logger().info("Connexion MAVLink établie !")
        
        ### IN MAV
        self.publisher_GPS_fire_coor = self.create_publisher(GeoPoint, 'IN/GPS_fire_coor', 10)
        self.timer = self.create_timer(0.01, self.tc_GPS_fire_coor)

        self.get_logger().info("NODE MAV_manager STARTED.")

    def tc_GPS_fire_coor(self):
        text_msg = self.mavlink_connection.recv_match()
        # self.get_logger().info('RECEIVED > %s' % text_msg)
        if not text_msg: # Obligatoire pour passer si c'est du bruit
            return
        if text_msg.get_type()=='STATUSTEXT':
            self.get_logger().info('RECEIVED > %s' % text_msg.text)
            if 'FIRE' in text_msg.text:
                # Create a GeoPoint message
                data = text_msg.text.split()
                geopoint_msg = GeoPoint()
                geopoint_msg.latitude = float(data[1])
                geopoint_msg.longitude = float(data[3])
                geopoint_msg.altitude = float(data[5])

                # Publish the GeoPoint message
                self.publisher_GPS_fire_coor.publish(geopoint_msg)  

def main(args=None):
    rclpy.init(args=args)
    node = MAVManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
