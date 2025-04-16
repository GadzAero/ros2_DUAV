#!/usr/bin/env python3
# Callback groups https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html

# General importation
from time import sleep
import time
import asyncio

# Import ROS2 utils
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

# Import MAVLink utils
import pymavlink.dialects.v20.common as mavlink
from pymavlink import mavutil
from pymavlink.mavutil import mavlink as mavkit

# Import Intefaces
from interfaces.srv import SetMode, Arm, Takeoff, Reposition, Land, Rtl, Disarm
from interfaces.action import TakeoffAct

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
            self.mav_master = mavutil.mavlink_connection('tcp:127.0.0.1:5782', baud=115200)
            # PX4 SITL
            # self.mav_master = mavutil.mavlink_connection('udp:127.0.0.1:14542')
            # RADIO
            # self.mav_master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        except Exception as e:
            self.get_logger().error(f"Erreur MAVLink : {e}")
            raise RuntimeError("Impossible de se connecter à MAVLink.")
        self.get_logger().warning("En attente du heartbeat MAVLink...")
        self.mav_master.wait_heartbeat()
        self.get_logger().info("Connexion MAVLink établie !")

        ### Request data stream from MAVLINK
        self.mav_master.mav.command_long_send(
            self.mav_master.target_system, self.mav_master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            245, 1_000_000, 0, 0, 0, 0, 0)
        self.mav_master.mav.request_data_stream_send(
            1, 
            mavutil.mavlink.MAV_COMP_ID_ALL,  
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 
            10,  # Fréquence de mise à jour (Hz)
            1  # Activer le flux (0 pour désactiver)
        )
        
        ### TIMER CALLBACK (it must run in paralell of services and actions but running it concurently to itself is useless)
        test1 = rclpy.callback_groups.ReentrantCallbackGroup()
        self.timer__status_text        = self.create_timer(0.1, self.timer_cb__status_text,        callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        self.timer__extended_sys_state = self.create_timer(0.1, self.timer_cb__extended_sys_state, callback_group=test1)
        
        ### SERVICES (running them concurently to themselves or actions is useless)
        self.srv__set_mode   = self.create_service(SetMode,    'set_mode',   self.srv_cb__set_mode,   callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        self.srv__arm        = self.create_service(Arm,        'arm',        self.srv_cb__arm,        callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        self.srv__takeoff    = self.create_service(Takeoff,    'takeoff',    self.srv_cb__takeoff,    callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        self.srv__reposition = self.create_service(Reposition, 'reposition', self.srv_cb__reposition, callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        self.srv__land       = self.create_service(Land,       'land',       self.srv_cb__land,       callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        self.srv__rtl        = self.create_service(Rtl,        'rtl',        self.srv_cb__rtl,        callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        self.srv__disarm     = self.create_service(Disarm,     'disarm',     self.srv_cb__disarm,     callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        
        ### ACTIONS (running them concurently to themselves or services is useless)
        test = rclpy.callback_groups.ReentrantCallbackGroup()
        self.act__takeoff = ActionServer(self, TakeoffAct, 'takeoff_act', self.act_cb__takeoff, callback_group=test)
        
        ### General Parameters
        self.fire_pos_lat = None
        self.fire_pos_lon = None
        self.landed_state = None
        
        self.get_logger().info("NODE MAV_manager STARTED.")
        
    ############################################################################################################################################################################################################################
    ##### TIMER CALLBACK ############################################################################################################################################################################################################################
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to reiceive STATUS_TEXT  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def timer_cb__status_text(self):
        text_msg = self.mav_master.recv_match(type="STATUSTEXT", blocking=True)
        if text_msg is not None: # For noise
            self.get_logger().info('RECEIVED > %s' % text_msg.text)
            if 'FIRE' in text_msg.text:
                # Create a GeoPoint message
                data = text_msg.text.split()
                self.fire_pos_lat = float(data[1])
                self.fire_pos_lon = float(data[3])
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to reiceive EXTENDED_SYS_STATE  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def timer_cb__extended_sys_state(self):
        landed_msg = self.mav_master.recv_match(type='EXTENDED_SYS_STATE', blocking=True)
        if landed_msg is not None:
            self.landed_state = landed_msg.landed_state
            if self.landed_state == 0:
                self.landed_state = "MAV_LANDED_STATE_UNDEFINED"
            elif self.landed_state == 1:
                self.landed_state = "MAV_LANDED_STATE_ON_GROUND"
            elif self.landed_state == 2:
                self.landed_state = "MAV_LANDED_STATE_IN_AIR"
            elif self.landed_state == 3:
                self.landed_state = "MAV_LANDED_STATE_TAKEOFF"
            elif self.landed_state == 4:
                self.landed_state = "MAV_LANDED_STATE_LANDING"

    ############################################################################################################################################################################################################################
    ##### ACTIONS CALLBACK ############################################################################################################################################################################################################################
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Action server to takeoff  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def act_cb__takeoff(self, goal_handle):
        print()
        self.get_logger().info(f"> Call action TAKEOFF (Alt:{goal_handle.request.alt})")
        
        feedback_msg = TakeoffAct.Feedback()
        result = TakeoffAct.Result()
        
        #### Taking off and returning the feedback
        feedback_msg.current_alt = 0.
        
        ## Check if the command is valid
        mav_utils.mav_takeoff(self.mav_master, goal_handle.request.alt*1.)
        # if not mav_utils.mav_takeoff(self.mav_master, goal_handle.request.alt*1.):
            # self.get_logger().warning("      -> Failure")
            # sleep(1)
            # goal_handle.abort()
            # result.success = False
            # return result
        
        start_time = time.time()
        elapsed_time = 0
        while elapsed_time < 10: # self.landed_state != "MAV_LANDED_STATE_IN_AIR" or
            self.get_logger().info(f"Taking off (Current_alt: {feedback_msg.current_alt}, State:{self.landed_state}, Elapsed_time:{elapsed_time:.1f})")
            sleep(1)
            elapsed_time = time.time() - start_time
            
        # if elapsed_time < 10:
        #     self.get_logger().warning("      -> Failure")
        #     goal_handle.abort()
        #     result.success = False
        #     return result
            
        self.get_logger().info("      -> Success.")
        goal_handle.succeed()
        result.success = True
        return result
             
    ############################################################################################################################################################################################################################
    ##### SERVICES CALLBACK ############################################################################################################################################################################################################################
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Service server to SET MODE ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def srv_cb__set_mode(self, request, response):
        print()
        self.get_logger().info(f"> Call service SET_MODE (Mode:{request.mode_name})")
        if mav_utils.mav_set_mode(self.mav_master, request.mode_name):
            response.success = True
            self.get_logger().info("      -> Success.")
        else:
            response.success = False
            self.get_logger().warning("      -> Failure")
        return response
    #--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Service server to ARM ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def srv_cb__arm(self, request, response):
        print()
        self.get_logger().info(f"> Call service ARM (Force:{request.force})")
        if mav_utils.mav_arm(self.mav_master, request.force):
            response.success = True
            self.get_logger().info("      -> Success.")
        else:
            response.success = False
            self.get_logger().warning("      -> Failure")
        return response
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Service server to TAKEOFF ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def srv_cb__takeoff(self, request, response):
        print()
        self.get_logger().info(f"> Call service TAKEOFF (Alt:{request.alt})")
        if mav_utils.mav_takeoff(self.mav_master, request.alt):
            response.success = True
            self.get_logger().info("      -> Success.")
        else:
            response.success = False
            self.get_logger().warning("      -> Failure")
        return response
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Service server to REPOSITION ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def srv_cb__reposition(self, request, response):
        print()
        self.get_logger().info(f"> Call service REPOSITION (lat:{request.lat}, lon:{request.lon}, Alt:{request.alt})")
        if mav_utils.mav_reposition(self.mav_master, request.lat, request.lon, request.alt):
            response.success = True
            self.get_logger().info("      -> Success.")
        else:
            response.success = False
            self.get_logger().warning("      -> Failure")
        return response
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Service server to LAND ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def srv_cb__land(self, request, response):
        print()
        self.get_logger().info(f"> Call service LAND.")
        if mav_utils.mav_land(self.mav_master):
            response.success = True
            self.get_logger().info("      -> Success.")
        else:
            response.success = False
            self.get_logger().warning("      -> Failure")
        return response
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Service server to RTL ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def srv_cb__rtl(self, request, response):
        print()
        self.get_logger().info(f"> Call service RTL.")
        if mav_utils.mav_rtl(self.mav_master):
            response.success = True
            self.get_logger().info("      -> Success.")
        else:
            response.success = False
            self.get_logger().warning("      -> Failure")
        return response
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Service server to DISARM ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def srv_cb__disarm(self, request, response):
        print()
        self.get_logger().info(f"> Call service DISARM (Force:{request.force})")
        if mav_utils.mav_disarm(self.mav_master, request.force):
            response.success = True
            self.get_logger().info("      -> Success.")
        else:
            response.success = False
            self.get_logger().warning("      -> Failure")
        return response

############################################################################################################################################################################################################################
##### Node entry point ############################################################################################################################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    
    ### Creating the mutlithread executor
    node = MAVManager()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=10)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
