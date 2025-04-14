#!/usr/bin/env python3

# General importation
from time import sleep

# Import ROS2 utils
import rclpy
from rclpy.node import Node

# Import MAVLink utils
import pymavlink.dialects.v20.common as mavlink
from pymavlink import mavutil
from pymavlink.mavutil import mavlink as mavkit
from geographic_msgs.msg import GeoPoint

# Import Intefaces
from interfaces.srv import SetMode

# Import MAV utils
import popeye.utils_MAV as mav_utils


############################################################################################################################################################################################################################
##### Node MAVLink Manager ############################################################################################################################################################################################################################
class MAVManager(Node):
    def __init__(self):
        super().__init__('MAV_manager', namespace='POPEYE')

        ### Connexion to MAVLink
        try:
            # ADRUPILOT SITL
            self.mav_master = mavutil.mavlink_connection('tcp:127.0.0.1:5780', baud=115200)
            # PX4 SITL
            # self.mav_master = mavutil.mavlink_connection('udp:127.0.0.1:14542')
            # RADIO
            # self.mav_master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        except Exception as e:
            self.get_logger().error(f"Erreur MAVLink : {e}")
            raise RuntimeError("Impossible de se connecter à MAVLink.")
        self.get_logger().info("En attente du heartbeat MAVLink...")
        self.mav_master.wait_heartbeat()
        self.get_logger().info("Connexion MAVLink établie !")

        ### Request data stream from MAVLINK
        self.mav_master.mav.request_data_stream_send(
            1, 
            mavutil.mavlink.MAV_COMP_ID_ALL,  
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 
            10,  # Fréquence de mise à jour (Hz)
            1  # Activer le flux (0 pour désactiver)
        )
        
        ### PUBLISHER
        self.publisher_GPS_fire_coor = self.create_publisher(GeoPoint, 'IN/GPS_fire_coor', 10)
        self.timer = self.create_timer(0.1, self.tc_GPS_fire_coor) # timer_callback
        
        ### SERVICES
        self.srv__set_mode = self.create_service(SetMode, 'set_mode', self.srv__set_mode_cb)
        
        ### General Parameters
        target_coor_received = False
        
        self.get_logger().info("NODE MAV_manager STARTED.")
        
        ### FIREFIGHTER MISSION
        # mav_utils.mav_set_mode(self.mav_master, 'AUTO')
        # mav_utils.mav_arm(self.mav_master)
        # mav_utils.mav_takeoff(self.mav_master, 6)
        # sleep(10)
        # mav_utils.mav_reposition(self.mav_master, 48.6128634, 2.3959637, 10)
        # sleep(20)
        # mav_utils.mav_rtl(self.mav_master)

    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to reiceive fire coor  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def tc_GPS_fire_coor(self):
        text_msg = self.mav_master.recv_match(type="STATUSTEXT", blocking=False)
        if text_msg is None: # For noise
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
            target_coor_received = True
            
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Service to CHANGE MODE ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def srv__set_mode_cb(self, request, response):
        self.get_logger().info(f"[SRV_SET_MODE] REQUEST > {request.mode_name}")
        if mav_utils.mav_set_mode(self.mav_master, request.mode_name):
            response.success = True
            self.get_logger().info("[SRV_SET_MODE] RESPONSE > Success.")
        else:
            response.success = False
            self.get_logger().warning("[SRV_SET_MODE] RESPONSE > Failure")
        return response

############################################################################################################################################################################################################################
##### Node entry point ############################################################################################################################################################################################################################
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
