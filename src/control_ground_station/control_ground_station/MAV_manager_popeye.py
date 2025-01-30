#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
import pymavlink.dialects.v20.common as mavlink
from pymavlink import mavutil

class MAVManagerPopeye(Node):
    def __init__(self):
        super().__init__('MAV_manager_popeye', namespace='CGS')

        # Connexion à MAVLink
        try:
            # self.mavlink_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5864') 
            self.mavlink_connection = mavutil.mavlink_connection('/dev/ttyUSB2', baud=57600)
            # self.mavlink_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
        except:
            self.get_logger().error("Impossible de se connecter à MAVLink")
        # finally:
            return
        self.get_logger().info("En attente du heartbeat MAVLink...")
        self.mavlink_connection.wait_heartbeat()
        self.get_logger().info("Connexion MAVLink établie !")
        
        ### IN MAV
        self.subscription_GPS_fire_coor = self.create_subscription(GeoPoint, 'GPS_fire_coor', self.lc_GPS_fire_coor, 10)

        self.get_logger().info("NODE MAV_manager STARTED.")

    def lc_GPS_fire_coor(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MAVManagerPopeye()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
