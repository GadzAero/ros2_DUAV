#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
import pymavlink.dialects.v20.common as mavlink
from pymavlink import mavutil

class MAVManagerOlive(Node):
    def __init__(self):
        super().__init__('MAV_manager_olive', namespace='CGS')

        # Connexion à MAVLink
        is_sitl = True
        try:
            if is_sitl:
                self.mavlink_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
            else:
                self.mavlink_connection = mavutil.mavlink_connection('/dev/ttyUSB2', baud=57600)
        except:
            self.get_logger().error("Impossible de se connecter à MAVLink")
        # finally:
            return
        self.get_logger().info("En attente du heartbeat MAVLink...")
        self.mavlink_connection.wait_heartbeat()
        self.get_logger().info("Connexion MAVLink établie !")
        
        ### OUT MAV
        self.publisher_GPS_fire_coor = self.create_publisher(GeoPoint, 'GPS_fire_coor', 10)
        self.timer = self.create_timer(1.0, self.tc_GPS_fire_coor)

        self.get_logger().info("NODE MAV_manager STARTED.")

    def tc_GPS_fire_coor(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MAVManagerOlive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
