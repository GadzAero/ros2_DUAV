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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

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
            245, 1e6, 0, 0, 0, 0, 0)
        self.mav_master.mav.command_long_send(
            self.mav_master.target_system, self.mav_master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            253, 0.5e6, 0, 0, 0, 0, 0)
        # self.mav_master.mav.request_data_stream_send(
        #     1, 
        #     mavutil.mavlink.MAV_COMP_ID_ALL,  
        #     mavutil.mavlink.MAV_DATA_STREAM_ALL, 
        #     10,  # Fréquence de mise à jour (Hz)
        #     1  # Activer le flux (0 pour désactiver)
        # )
        
        ### TIMER CALLBACK (it must run in paralell of services and actions but running it concurently to itself is useless)
        self.timer__all = self.create_timer(0.05, self.timer_cb__all, callback_group=MutuallyExclusiveCallbackGroup())
        
        ### SERVICES (running them concurently to themselves or actions is useless)
        self.srv__set_mode   = self.create_service(SetMode,    'set_mode',   self.srv_cb__set_mode,   callback_group=MutuallyExclusiveCallbackGroup())
        self.srv__arm        = self.create_service(Arm,        'arm',        self.srv_cb__arm,        callback_group=MutuallyExclusiveCallbackGroup())
        # self.srv__takeoff    = self.create_service(Takeoff,    'takeoff',    self.srv_cb__takeoff,    callback_group=MutuallyExclusiveCallbackGroup())
        self.srv__reposition = self.create_service(Reposition, 'reposition', self.srv_cb__reposition, callback_group=MutuallyExclusiveCallbackGroup())
        self.srv__land       = self.create_service(Land,       'land',       self.srv_cb__land,       callback_group=MutuallyExclusiveCallbackGroup())
        self.srv__rtl        = self.create_service(Rtl,        'rtl',        self.srv_cb__rtl,        callback_group=MutuallyExclusiveCallbackGroup())
        self.srv__disarm     = self.create_service(Disarm,     'disarm',     self.srv_cb__disarm,     callback_group=MutuallyExclusiveCallbackGroup())
        
        ### ACTIONS (running them concurently to themselves or services is useless)
        self.act__takeoff = ActionServer(self, TakeoffAct, 'takeoff_act', self.act_cb__takeoff, callback_group=ReentrantCallbackGroup())
        
        ### General Parameters
        self.fire_pos_lat = None
        self.fire_pos_lon = None
        self.nb_msg = 0
        self.landed_state = ""
        self.landed_state2 = ""
        
        ### Timers for debug 
        self.elapsed_time = -time.time()
        self.start_time = time.time()
        
        self.get_logger().info("NODE MAV_manager STARTED.")
        
    ############################################################################################################################################################################################################################
    ##### TIMER CALLBACK ############################################################################################################################################################################################################################
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to reiceive ALL  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def timer_cb__all(self):
        ### Receive Mavlink messages
        msg = self.mav_master.recv_match(type=['EXTENDED_SYS_STATE', 'STATUSTEXT'], blocking=False)
        ## For noise
        if not msg:
            return
        
        ### Save the message data
        if msg.get_type() == "STATUSTEXT":
            self.get_logger().info('RECEIVED > %s' % msg.text)
            if 'FIRE' in msg.text:
                # Create a GeoPoint message
                data = msg.text.split()
                self.fire_pos_lat = float(data[1])
                self.fire_pos_lon = float(data[3])
        elif msg.get_type() == "EXTENDED_SYS_STATE":
            landed_state_id = msg.landed_state
            if landed_state_id == 0:
                self.landed_state = "MAV_LANDED_STATE_UNDEFINED"
            elif landed_state_id == 1:
                self.landed_state = "MAV_LANDED_STATE_ON_GROUND"
            elif landed_state_id == 2:
                self.landed_state = "MAV_LANDED_STATE_IN_AIR"
            elif landed_state_id == 3:
                self.landed_state = "MAV_LANDED_STATE_TAKEOFF"
            elif landed_state_id == 4:
                self.landed_state = "MAV_LANDED_STATE_LANDING"
            # Bench the perf
            self.nb_msg+=1
            self.elapsed_time += time.time()
            print(f"OKKKK: {self.landed_state} --- time:{self.elapsed_time:.5f} --- Nb msg:{self.nb_msg}/{(time.time()-self.start_time)/1:.0f}")
            self.elapsed_time = -time.time()

    ############################################################################################################################################################################################################################
    ##### ACTIONS CALLBACK ############################################################################################################################################################################################################################
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Action server to takeoff  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def act_cb__takeoff(self, goal_handle):
        print()
        self.get_logger().info(f"> Call action TAKEOFF (Alt:{goal_handle.request.alt})")
        
        feedback_msg = TakeoffAct.Feedback()
        result = TakeoffAct.Result()
        
        
        self.get_logger().info(f"> 1111111111")
        
        #### Taking off and returning the feedback
        feedback_msg.current_alt = 0.
        
        self.get_logger().info(f"> 222222222222")
        
        ## Check if the command is valid
        if not mav_utils.mav_takeoff(self.mav_master, goal_handle.request.alt*1.):
            self.get_logger().warning("      -> Failure")
            sleep(1)
            goal_handle.abort()
            result.success = False
            return result
        
        self.get_logger().info(f"33333333333333")
        
        start_time = time.time()
        elapsed_time = 0
        while elapsed_time < 10: # self.landed_state != "MAV_LANDED_STATE_IN_AIR" or
            self.get_logger().info(f"Taking off (Current_alt: {feedback_msg.current_alt}, State:{10}, Elapsed_time:{elapsed_time:.1f})")
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
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
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
