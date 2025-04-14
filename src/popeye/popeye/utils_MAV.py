# Standard commands : https://mavlink.io/en/services/command.html

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

# PARAMETERS
SGDB_LAT = 48.6126523
SGDB_LON = 2.3963258
DUAV_LAT = 45.4389466
DUAV_LON = -0.4283327
DEFAULT_LAT = SGDB_LAT
DEFAULT_LON = SGDB_LON
DEFAULT_ALT = 6

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
        
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#----- Function to ARM ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def mav_arm(master, force=False):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavkit.MAV_CMD_COMPONENT_ARM_DISARM, 0, 
        1, 21196 if force else 0, 0, 0, 0, 0, 0)
    ### Receive acknologment
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
    if not ack_msg['result']==mavkit.MAV_RESULT_ACCEPTED:
        print(f"{RED}[MAV_ARM] " + mavkit.enums['MAV_RESULT'][ack_msg['result']].description)
        return False
    return True

#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#----- Function to TAKEOFF ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def mav_takeoff(master, alt=6):
    ### Send the command
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavkit.MAV_CMD_NAV_TAKEOFF, 0, 
        0, 0, 0, 0, 0, 0, alt)
    ### Receive acknologment
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
    if not ack_msg['result']==mavkit.MAV_RESULT_ACCEPTED:
        print(f"{RED}[MAV_TAKEOFF] " + mavkit.enums['MAV_RESULT'][ack_msg['result']].description)
        return False
    return True

#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#----- Function to REPOSITION --------------------------------------------------------------------------------------------------------------------------------------------------------------------
def mav_reposition(master, lat=DEFAULT_LAT, lon=DEFAULT_LON, alt=DEFAULT_ALT, local=False):
    ### Send the command
    if local:
        lat = int(lat*10**4)
        lon = int(lon*10**4)
        frame = mavkit.MAV_FRAME_LOCAL_NED
    else:
        lat = int(lat*10**7)
        lon = int(lon*10**7)
        frame = mavkit.MAV_FRAME_GLOBAL_RELATIVE_ALT
    master.mav.command_int_send(
        master.target_system, master.target_component, 
        frame, mavkit.MAV_CMD_DO_REPOSITION, 0, 0,
        0, 1, 0, 0, lat, lon, alt)
    ### Receive acknologment
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
    if not ack_msg['result']==mavkit.MAV_RESULT_ACCEPTED:
        print(f"{RED}[MAV_REPOSITION] " + mavkit.enums['MAV_RESULT'][ack_msg['result']].description)
        return False
    return True

#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#----- Function to LAND ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def mav_land(master):
    ### Send the command
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavkit.MAV_CMD_NAV_LAND, 0, 
        0, 0, 0, 0, 0, 0, 0)
    ### Receive acknologment
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
    if not ack_msg['result']==mavkit.MAV_RESULT_ACCEPTED:
        print(f"{RED}[MAV_LAND] " + mavkit.enums['MAV_RESULT'][ack_msg['result']].description)
        return False
    return True

#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#----- Function to RTL ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def mav_rtl(master):
    ### Send the command
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavkit.MAV_CMD_NAV_RETURN_TO_LAUNCH , 0, 
        0, 0, 0, 0, 0, 0, 0)
    ### Receive acknologment
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
    if not ack_msg['result']==mavkit.MAV_RESULT_ACCEPTED:
        print(f"{RED}[MAV_RTL] " + mavkit.enums['MAV_RESULT'][ack_msg['result']].description)
        return False
    return True

#-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#----- Function to DISARM ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def mav_disarm(master, force=False):
    ### Send the command
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavkit.MAV_CMD_COMPONENT_ARM_DISARM, 0, 
        0, 21196 if force else 0, 0, 0, 0, 0, 0)
    ### Receive acknologment
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
    if not ack_msg['result']==mavkit.MAV_RESULT_ACCEPTED:
        print(f"{RED}[MAV_DISARM] " + mavkit.enums['MAV_RESULT'][ack_msg['result']].description)
        return False
    return True