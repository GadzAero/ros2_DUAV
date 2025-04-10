#!/usr/bin/env python3

<<<<<<< HEAD
=======
from geographic_msgs.msg import GeoPoint
# from interfaces.srv import TriggerArm

>>>>>>> 4c497ca (UPDATE: Can send but not received)
import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint

# Import PyMavLink
import pymavlink.dialects.v20.common as mavlink
from pymavlink import mavutil
from pymavlink.mavutil import mavlink as mavkit

#####################################################################################################################################################################
##### Node MAVLink Manager #####################################################################################################################################################################
class MAVManager(Node):
    def __init__(self):
        super().__init__('MAV_manager', namespace='POPEYE')

        ### Connexion to MAVLink
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

        ### Request data stream
        self.mavlink_connection.mav.request_data_stream_send(
            1, 
            mavutil.mavlink.MAV_COMP_ID_ALL,  
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 
            10,  # Fréquence de mise à jour (Hz)
            1  # Activer le flux (0 pour désactiver)
        )
        
        ### IN/MAV
        self.publisher_GPS_fire_coor = self.create_publisher(GeoPoint, 'IN/GPS_fire_coor', 10)
        self.timer = self.create_timer(0.5, self.tc_GPS_fire_coor) # timer_callback
<<<<<<< HEAD
=======
        
        ### SERVICES
        # self.srv = self.create_service(TriggerArm, 'trigger_arm', self.arm)
>>>>>>> 4c497ca (UPDATE: Can send but not received)

        self.get_logger().info("NODE MAV_manager STARTED.")

    #---------------------------------------------------------------------------------------------------------------------------------
    #----- Function to reiceive fire coor  ------------------------------------------------------------------------------------
    def tc_GPS_fire_coor(self):
        text_msg = self.mavlink_connection.recv_match(type="STATUSTEXT", blocking=False)
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
            
    #---------------------------------------------------------------------------------------------------------------------------------
    #----- Function to ARM ------------------------------------------------------------------------------------
    def mav_arm(master, force=False):
        force_id = 0
        if force:
            force_id = 21196
        ### Send the command
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavkit.MAV_CMD_COMPONENT_ARM_DISARM, 0, 
            1, force_id, 0, 0, 0, 0, 0)
        ### Receive acknologment
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
        if not ack_msg['result']==mavkit.MAV_RESULT_ACCEPTED:
            print("[COMMAND ARM] " + mavkit.enums['MAV_RESULT'][ack_msg['result']].description)
            return False
        print("[COMMAND ARM] Arming (forced:"+str(force)+").")
        return True
    
    #---------------------------------------------------------------------------------------------------------------------------------
    #----- Function to DISARM ------------------------------------------------------------------------------------
    def mav_disarm(master, force=False):
        force_id = 0
        if force:
            force_id = 21196
        ### Send the command
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavkit.MAV_CMD_COMPONENT_ARM_DISARM, 0, 
            0, force_id, 0, 0, 0, 0, 0)
        ### Receive acknologment
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
        if not ack_msg['result']==mavkit.MAV_RESULT_ACCEPTED:
            print("[COMMAND DISARM] " + mavkit.enums['MAV_RESULT'][ack_msg['result']].description)
            return False
        print("[COMMAND DISARM] Disarming (forced:"+str(force)+").")
        return True

<<<<<<< HEAD
    #---------------------------------------------------------------------------------------------------------------------------------
    #----- Function to CHANGE MODE ------------------------------------------------------------------------------------def mav_set_mode(master, mode):
    def mav_set_mode(master, mode):
        ### Check if the mode is valid
        if mode not in master.mode_mapping():
            print("[COMMAND SET MODE] Unknown mode.")
            print("[COMMAND SET MODE] Valid modes: ", list(master.mode_mapping().keys()))
            return False
        ### Send the command
        mode_id = master.mode_mapping()[mode]
        master.set_mode(mode_id)
        ### Receive acknologment
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
        if not ack_msg['result']==mavkit.MAV_RESULT_ACCEPTED:
            print("[COMMAND SET MODE] " + mavkit.enums['MAV_RESULT'][ack_msg['result']].description)
            return False
        print("[COMMAND SET MODE] Mode changed > " + mode)
        return True
    
    #---------------------------------------------------------------------------------------------------------------------------------
    #----- Function to LAND ------------------------------------------------------------------------------------
    def mav_land(master):
        ### Send the command
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavkit.MAV_CMD_NAV_LAND, 0, 
            0, 0, 0, 0, 0, 0, 0)
        ### Receive acknologment
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
        if not ack_msg['result']==mavkit.MAV_RESULT_ACCEPTED:
            print("[COMMAND LAND] " + mavkit.enums['MAV_RESULT'][ack_msg['result']].description)
            return False
        print("[COMMAND LAND] Landing.")
        return True
=======
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
>>>>>>> 4c497ca (UPDATE: Can send but not received)

    #---------------------------------------------------------------------------------------------------------------------------------
    #----- Function to RTL ------------------------------------------------------------------------------------
    def mav_rtl(master):
        ### Send the command
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavkit.MAV_CMD_NAV_RETURN_TO_LAUNCH , 0, 
            0, 0, 0, 0, 0, 0, 0)
        ### Receive acknologment
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
        if not ack_msg['result']==mavkit.MAV_RESULT_ACCEPTED:
            print("[COMMAND RTL] " + mavkit.enums['MAV_RESULT'][ack_msg['result']].description)
            return False
        print("[COMMAND RTL] Return To Launch.")
        return True

    #---------------------------------------------------------------------------------------------------------------------------------
    #----- Function to REPOSITION ------------------------------------------------------------------------------------
    def mav_reposition(master, lat=45.4389466, lon=-0.4283327, alt=10, local=False):
        ### Send the command
        if local:
            lat = int(lat*10**4)
            lon = int(lon*10**4)
            frame = mavkit.MAV_FRAME_LOCAL_NED
            print(frame)
        else:
            lat = int(lat*10**7)
            lon = int(lon*10**7)
            frame = mavkit.MAV_FRAME_GLOBAL_RELATIVE_ALT
        print(frame)
        master.mav.command_int_send(
            master.target_system, master.target_component, 
            frame, mavkit.MAV_CMD_DO_REPOSITION, 0, 0,
            0, 1, 0, 0, lat, lon, alt)
        ### Receive acknologment
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
        if not ack_msg['result']==mavkit.MAV_RESULT_ACCEPTED:
            print("[COMMAND REPOSITION] " + mavkit.enums['MAV_RESULT'][ack_msg['result']].description)
            return False
        print(f"[COMMAND REPOSITION] Repositioning to WP ({int(lat*10**7)}, {int(lon*10**7)}, {alt}).")
        return True
            

#####################################################################################################################################################################
##### Node entry point #####################################################################################################################################################################
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