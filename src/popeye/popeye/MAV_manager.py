#!/usr/bin/env python3

from geographic_msgs.msg import GeoPoint
# from interfaces.srv import TriggerArm

import rclpy
from rclpy.node import Node
import pymavlink.dialects.v20.common as mavlink
from pymavlink import mavutil

class MAVManager(Node):
    def __init__(self):
        super().__init__('MAV_manager', namespace='POPEYE')

        # Connexion à MAVLink
        try:
            # ADRUPILOT SITL
            self.mavlink_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5780', baud=115200)
            # PX4 SITL
            # self.mavlink_connection = mavutil.mavlink_connection('udp:127.0.0.1:14542')
            # RADIO
            # self.mavlink_connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        except Exception as e:
            self.get_logger().error(f"Erreur MAVLink : {e}")
            raise RuntimeError("Impossible de se connecter à MAVLink.")
        self.get_logger().info("En attente du heartbeat MAVLink...")
        self.mavlink_connection.wait_heartbeat()
        self.get_logger().info("Connexion MAVLink établie !")

        self.mavlink_connection.mav.request_data_stream_send(
            1,  # Target System ID
            mavutil.mavlink.MAV_COMP_ID_ALL,  # Target Component ID
            mavutil.mavlink.MAV_DATA_STREAM_ALL,  # Type de données
            10,  # Fréquence de mise à jour (Hz)
            1  # Activer le flux (0 pour désactiver)
        )
        
        ### IN/MAV
        self.publisher_GPS_fire_coor = self.create_publisher(GeoPoint, 'IN/GPS_fire_coor', 10)
        self.timer = self.create_timer(0.5, self.tc_GPS_fire_coor) # timer_callback
        
        ### SERVICES
        # self.srv = self.create_service(TriggerArm, 'trigger_arm', self.arm)

        self.get_logger().info("NODE MAV_manager STARTED.")

    def tc_GPS_fire_coor(self):
        text_msg = self.mavlink_connection.recv_match(type="STATUSTEXT", blocking=False)
        if text_msg is None: # Obligatoire pour passer si c'est du bruit
            return
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

    # Function to make Olive takeoff
    # def arm(self, request, response):
    #     try:
    #         self.mavlink_connection.arducopter_arm()
    #         self.mavlink_connection.motors_armed_wait()
    #         response.success = True
    #     except Exception as e:
    #         self.get_logger().error(f"Failed to takeoff: {str(e)}")
    #         self.get_logger().info("Popeye ARM FAILED")
    #         response.success = False
    #     self.get_logger().info("Popeye ARMED")
    #     return response

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