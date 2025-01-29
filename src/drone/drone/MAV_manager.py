#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
import pymavlink.dialects.v20.common as mavlink
from pymavlink import mavutil

class MAVManager(Node):
    def __init__(self):
        super().__init__('MAV_manager', namespace='OLIVE')

        # Connexion à MAVLink
        try:
            self.mavlink_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5964') 
            # self.mavlink_connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
            # self.mavlink_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
        except:
            self.get_logger().error("Impossible de se connecter à MAVLink")
        # finally:
            return
        self.get_logger().info("En attente du heartbeat MAVLink...")
        self.mavlink_connection.wait_heartbeat()
        self.get_logger().info("Connexion MAVLink établie !")

        ### OUT MAV
        self.subscription_GPS_fire_coor = self.create_subscription(GeoPoint, 'OUT/GPS_fire_coor', self.lc_GPS_fire_coor, 10)
        
        ### IN MAV
        self.publisher_GPS_olive_coor = self.create_publisher(GeoPoint, 'IN/GPS_olive_coor', 10)
        self.timer = self.create_timer(1.0, self.tc_GPS_olive_coor)

        self.get_logger().info("NODE MAV_manager STARTED.")

    def lc_GPS_fire_coor(self, GPS_fire_coor):
        # Log the received coordinates
        # self.get_logger().info(f"Published: Latitude={GPS_fire_coor.latitude}, Longitude={GPS_fire_coor.longitude}, Altitude={GPS_fire_coor.altitude}")
        
        # Format the coordinates into a string
        status_msg = f"FIRE AT - Lat:{GPS_fire_coor.latitude} Lon:{GPS_fire_coor.longitude} Alt:{GPS_fire_coor.altitude}"
        
        # Send the text message using STATUSTEXT
        self.mavlink_connection.mav.statustext_send(
            severity=mavutil.mavlink.MAV_SEVERITY_EMERGENCY,  # Severity level
            text=status_msg.encode('utf-8')             # Message text, encoded as UTF-8
        )
        
        # Log that the message was sent
        # self.get_logger().info("GPS coordinates sent as a STATUSTEXT message")

    def tc_GPS_olive_coor(self):
        try:
            # Wait for the GLOBAL_POSITION_INT message
            msg = self.mavlink_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if msg is not None:
                # Extract global position data
                latitude = msg.lat / 1e7  # Convert from 1E7 degrees to decimal degrees
                longitude = msg.lon / 1e7  # Convert from 1E7 degrees to decimal degrees
                altitude = msg.alt / 1000.0  # Convert from millimeters to meters

                # Create a GeoPoint message
                geopoint_msg = GeoPoint()
                geopoint_msg.latitude = latitude
                geopoint_msg.longitude = longitude
                geopoint_msg.altitude = altitude

                # Publish the GeoPoint message
                self.publisher_GPS_olive_coor.publish(geopoint_msg)
                self.get_logger().info(f"Published global position: Lat={latitude:.6f}, Lon={longitude:.6f}, Alt={altitude:.2f}m")
        except Exception as e:
            self.get_logger().error(f"Failed to read or publish global position: {str(e)}")

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
