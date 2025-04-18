#!/usr/bin/env python3
# Callback groups https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html

# General importation
from time import sleep
import time
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
from interfaces.srv import SetMode, Arm, Reposition, Rtl, Disarm
from interfaces.action import Takeoff, Land
# Import MAV utils
import popeye.utils_MAV as mav_utils

############################################################################################################################################################################################################################
##### Node MAVLink Manager ############################################################################################################################################################################################################################
class MAVManager(Node):
    def __init__(self):
        super().__init__('MAV_manager', namespace='POPEYE')
        self.get_logger().info("NODE MAV_manager STARTED.")

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
        ## STATUSTEXT
        self.mav_master.mav.command_long_send(
            self.mav_master.target_system, self.mav_master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            253, 1e6, 0, 0, 0, 0, 0)
        ## GLOBAL_POSITION_INT
        self.mav_master.mav.command_long_send(
            self.mav_master.target_system, self.mav_master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            33, 1e6, 0, 0, 0, 0, 0)
        ## EXTENDED_SYS_STATE
        self.mav_master.mav.command_long_send(
            self.mav_master.target_system, self.mav_master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            245, 1e6, 0, 0, 0, 0, 0)
        
        ### TIMER CALLBACK (it must run in paralell of services and actions but running it concurently to itself is useless)
        self.timer__all = self.create_timer(0.05, self.timer_cb__all, callback_group=MutuallyExclusiveCallbackGroup())
        
        ### SERVICES (running them concurently to themselves or actions is useless)
        self.srv__set_mode   = self.create_service(SetMode,    'set_mode',   self.srv_cb__set_mode,   callback_group=MutuallyExclusiveCallbackGroup())
        self.srv__arm        = self.create_service(Arm,        'arm',        self.srv_cb__arm,        callback_group=MutuallyExclusiveCallbackGroup())
        self.srv__reposition = self.create_service(Reposition, 'reposition', self.srv_cb__reposition, callback_group=MutuallyExclusiveCallbackGroup())
        self.srv__rtl        = self.create_service(Rtl,        'rtl',        self.srv_cb__rtl,        callback_group=MutuallyExclusiveCallbackGroup())
        self.srv__disarm     = self.create_service(Disarm,     'disarm',     self.srv_cb__disarm,     callback_group=MutuallyExclusiveCallbackGroup())
        
        ### ACTIONS (running them concurently to themselves or services is useless)
        self.act__takeoff = ActionServer(self, Takeoff, 'takeoff', self.act_cb__takeoff, callback_group=MutuallyExclusiveCallbackGroup())
        self.act__land    = ActionServer(self, Land,       'land', self.act_cb__land,    callback_group=MutuallyExclusiveCallbackGroup())
        
        ### General Parameters
        ## Fire position
        self.fire_pos_lat = None
        self.fire_pos_lon = None
        ## Popeye position
        self.popeye_pos_lat = None
        self.popeye_pos_lon = None
        self.popeye_pos_alt = None
        ## Popeye state
        self.nb_msg = 0
        self.landed_state = ""
        
        ### Timers for debug 
        self.elapsed_time = -time.time()
        self.start_time = time.time()
        
    ############################################################################################################################################################################################################################
    ##### TIMER CALLBACK ############################################################################################################################################################################################################################
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to reiceive ALL  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def timer_cb__all(self):
        ### Receive Mavlink messages
        msg = self.mav_master.recv_match(type=['EXTENDED_SYS_STATE', 'STATUSTEXT', 'GLOBAL_POSITION_INT'], blocking=False)
        if msg is None:
            return
        ## For debug
        # try:
        #     self.get_logger().info(f"{self.mav_master.recv_match().to_dict()}")
        # except:
        #     pass
        
        ### Save the data returning from the messages
        msg_type = msg.get_type()
        ## For TEXT messages
        if msg_type == "STATUSTEXT":
            self.get_logger().info('RECEIVED > %s' % msg.text)
            if 'FIRE' in msg.text:
                data = msg.text.split()
                self.fire_pos_lat = float(data[1])
                self.fire_pos_lon = float(data[3])
        ## For GLOBAL_POSITION_INT messages
        elif msg_type == "GLOBAL_POSITION_INT":
            self.popeye_pos_lat = msg.lat/1e7
            self.popeye_pos_lon = msg.lon/1e7
            self.popeye_pos_alt = msg.alt/1e3
            # self.get_logger().info(f"RECEIVED > Lat: {self.popeye_pos_lat} Lon: {self.popeye_pos_lon} Alt: {self.popeye_pos_alt}")
        ## For LANDED_STATE messages
        elif msg_type == "EXTENDED_SYS_STATE":
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
            ## Bench the perf
            # self.nb_msg+=1
            # self.elapsed_time += time.time()
            # print(f"LANDEDE_STATE:{self.landed_state} --- time:{self.elapsed_time:.5f} --- Nb msg:{self.nb_msg}/{(time.time()-self.start_time)/1:.0f}")
            # self.elapsed_time = -time.time()

    ############################################################################################################################################################################################################################
    ##### ACTIONS CALLBACK ############################################################################################################################################################################################################################
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Action server to TAKEOFF  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def act_cb__takeoff(self, goal_handle):
        print()
        self.get_logger().info(f"> Call action TAKEOFF (Alt:{goal_handle.request.alt})")
        
        if not mav_utils.mav_takeoff(self.mav_master, goal_handle.request.alt):
            self.get_logger().warning("      -> Failure: command not valid")
            goal_handle.abort()
            return Takeoff.Result(success=False)
        
        start_time = time.time()
        while self.landed_state != "MAV_LANDED_STATE_IN_AIR":
            goal_handle.publish_feedback(Takeoff.Feedback(current_alt=self.popeye_pos_alt, state=self.landed_state))
            self.get_logger().info(f"      ... Taking off (Current_alt:{self.popeye_pos_alt}, State:{self.landed_state})")
            if time.time()-start_time > 25:
                self.get_logger().warning("      -> Failure: command has timed out")
                goal_handle.abort()
                return Takeoff.Result(success=False)
            sleep(1)
        self.get_logger().info("      -> Success")
        goal_handle.succeed()
        return Takeoff.Result(success=True)
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Action server to LAND  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def act_cb__land(self, goal_handle):
        print()
        self.get_logger().info(f"> Call action LAND")
        
        if not mav_utils.mav_land(self.mav_master):
            self.get_logger().warning("      -> Failure: command not valid")
            goal_handle.abort()
            return Land.Result(success=False)
        
        start_time = time.time()
        while self.landed_state != "MAV_LANDED_STATE_ON_GROUND":
            goal_handle.publish_feedback(Land.Feedback(current_alt=self.popeye_pos_alt, state=self.landed_state))
            self.get_logger().info(f"      ... Landing (Current_alt:{self.popeye_pos_alt}, State:{self.landed_state})")
            if time.time()-start_time > 40:
                self.get_logger().warning("      -> Failure: command has timed out")
                goal_handle.abort()
                return Land.Result(success=False)
            sleep(1)
        self.get_logger().info("      -> Success")
        goal_handle.succeed()
        return Land.Result(success=True)
             
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
