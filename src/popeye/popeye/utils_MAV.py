# Import ROS2 utils
from rclpy.node import Node

# Import MAVLink utils
import pymavlink.dialects.v20.common as mavlink
from pymavlink import mavutil
from pymavlink.mavutil import mavlink as mavkit

# Colors from console print
RESET = "\033[0m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
MAGENTA = "\033[35m"
CYAN = "\033[36m"
WHITE = "\033[37m"
BOLD = "\033[1m"
UNDERLINE = "\033[4m"
REVERSED = "\033[7m"

#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#----- Function to CHANGE MODE ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def mav_set_mode(master, mode):
    ### Check if the mode is valid
    if mode not in master.mode_mapping():
        print(f"{RED}[MAV_SET_MODE] MODE UKNOWN. Valid modes: {list(master.mode_mapping().keys())}")
        return False
    ### Send the command
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)
    ### Receive acknologment
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
    if not ack_msg['result']==mavkit.MAV_RESULT_ACCEPTED:
        print(f"{RED}[MAV_SET_MODE] " + mavkit.enums['MAV_RESULT'][ack_msg['result']].description)
        return False
    return True
        
# #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# #----- Function to ARM ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# def mav_arm(master, force=False):
#     force_id = 0
#     if force:
#         force_id = 21196
#     ### Send the command
#     master.mav.command_long_send(
#         master.target_system, master.target_component,
#         mavkit.MAV_CMD_COMPONENT_ARM_DISARM, 0, 
#         1, force_id, 0, 0, 0, 0, 0)
#     ### Receive acknologment
#     ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
#     if not ack_msg['result']==mavkit.MAV_RESULT_ACCEPTED:
#         print("[COMMAND ARM] " + mavkit.enums['MAV_RESULT'][ack_msg['result']].description)
#         return False
#     print("[COMMAND ARM] Arming (forced:"+str(force)+").")
#     return True

# #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# #----- Function to TAKEOFF ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# def mav_takeoff(master, alt=6):
#     ### Send the command
#     master.mav.command_long_send(
#         master.target_system, master.target_component,
#         mavkit.MAV_CMD_NAV_TAKEOFF, 0, 
#         0, 0, 0, 0, 0, 0, alt)
#     ### Receive acknologment
#     ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
#     if not ack_msg['result']==mavkit.MAV_RESULT_ACCEPTED:
#         print("[COMMAND TAKEOFF] " + mavkit.enums['MAV_RESULT'][ack_msg['result']].description)
#         return False
#     print("[COMMAND TAKEOFF] Taking off.")
#     return True

# #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# #----- Function to REPOSITION --------------------------------------------------------------------------------------------------------------------------------------------------------------------
# def mav_reposition(master, lat=45.4389466, lon=-0.4283327, alt=10, local=False):
#     ### Send the command
#     if local:
#         lat = int(lat*10**4)
#         lon = int(lon*10**4)
#         frame = mavkit.MAV_FRAME_LOCAL_NED
#         print(frame)
#     else:
#         lat = int(lat*10**7)
#         lon = int(lon*10**7)
#         frame = mavkit.MAV_FRAME_GLOBAL_RELATIVE_ALT
#     print(frame)
#     master.mav.command_int_send(
#         master.target_system, master.target_component, 
#         frame, mavkit.MAV_CMD_DO_REPOSITION, 0, 0,
#         0, 1, 0, 0, lat, lon, alt)
#     ### Receive acknologment
#     ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
#     if not ack_msg['result']==mavkit.MAV_RESULT_ACCEPTED:
#         print("[COMMAND REPOSITION] " + mavkit.enums['MAV_RESULT'][ack_msg['result']].description)
#         return False
#     print(f"[COMMAND REPOSITION] Repositioning to WP ({int(lat*10**7)}, {int(lon*10**7)}, {alt}).")
#     return True

# #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# #----- Function to LAND ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# def mav_land(master):
#     ### Send the command
#     master.mav.command_long_send(
#         master.target_system, master.target_component,
#         mavkit.MAV_CMD_NAV_LAND, 0, 
#         0, 0, 0, 0, 0, 0, 0)
#     ### Receive acknologment
#     ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
#     if not ack_msg['result']==mavkit.MAV_RESULT_ACCEPTED:
#         print("[COMMAND LAND] " + mavkit.enums['MAV_RESULT'][ack_msg['result']].description)
#         return False
#     print("[COMMAND LAND] Landing.")
#     return True

# #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# #----- Function to RTL ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# def mav_rtl(master):
#     ### Send the command
#     master.mav.command_long_send(
#         master.target_system, master.target_component,
#         mavkit.MAV_CMD_NAV_RETURN_TO_LAUNCH , 0, 
#         0, 0, 0, 0, 0, 0, 0)
#     ### Receive acknologment
#     ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
#     if not ack_msg['result']==mavkit.MAV_RESULT_ACCEPTED:
#         print("[COMMAND RTL] " + mavkit.enums['MAV_RESULT'][ack_msg['result']].description)
#         return False
#     print("[COMMAND RTL] Return To Launch.")
#     return True

# #-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# #----- Function to DISARM ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# def mav_disarm(master, force=False):
#     force_id = 0
#     if force:
#         force_id = 21196
#     ### Send the command
#     master.mav.command_long_send(
#         master.target_system, master.target_component,
#         mavkit.MAV_CMD_COMPONENT_ARM_DISARM, 0, 
#         0, force_id, 0, 0, 0, 0, 0)
#     ### Receive acknologment
#     ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
#     if not ack_msg['result']==mavkit.MAV_RESULT_ACCEPTED:
#         print("[COMMAND DISARM] " + mavkit.enums['MAV_RESULT'][ack_msg['result']].description)
#         return False
#     print("[COMMAND DISARM] Disarming (forced:"+str(force)+").")
#     return True