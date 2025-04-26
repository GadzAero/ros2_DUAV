#!/usr/bin/env python3

# Import standard utils
from time import sleep
import threading
# Import ROS2 utils
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# Import Intefaces
from interfaces.srv import SetMode, Arm, Rtl, Disarm, Drop
from interfaces.action import Takeoff, Land, Reposition
from interfaces.msg import Fire
# Import FSM utils
import popeye.FSM__utils as fsm
from popeye.PARAMS_utils import *

#####################################################################################################################################################################
##### Node MAVLink Manager #####################################################################################################################################################################
class FSMInterfaceNode(Node):
    def __init__(self):
        super().__init__('FSM_interface_node', namespace='POPEYE')
        
        ### Global prarams
        self.cancel_action = False
        self.is_fire = False
        
        ### SERVICE CLIENTS
        self.cli_srv__set_mode = self.create_client(SetMode, 'set_mode', callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_srv__arm      = self.create_client(Arm,     'arm',      callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_srv__drop     = self.create_client(Drop,    'drop',     callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_srv__rtl      = self.create_client(Rtl,     'rtl',      callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_srv__disarm   = self.create_client(Disarm,  'disarm',   callback_group=MutuallyExclusiveCallbackGroup())
        
        ### ACTIONS CLIENTS
        self.cli_act__takeoff    = ActionClient(self, Takeoff,    'takeoff',    callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_act__land       = ActionClient(self, Land,       'land',       callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_act__reposition = ActionClient(self, Reposition, 'reposition', callback_group=MutuallyExclusiveCallbackGroup())
        
        ### SUBSCRIBERS 
        self.sub__fire = self.create_subscription(Fire, 'fire', self.sub_cb__fire, 10, callback_group=MutuallyExclusiveCallbackGroup())
        
        ### TIMERS 
        self.timer__fsm = self.create_timer(1, self.timer_cb__fsm, callback_group=MutuallyExclusiveCallbackGroup())
        
        self.get_logger().info(" > NODE FSM_interface STARTED.")
    
    ############################################################################################################################################################################################################################
    ##### TIMERS CALLBACKS ############################################################################################################################################################################################################################
    def timer_cb__fsm(self):
        ### Start the FSM
        if True:
            self.call__disarm(force=True)
            self.get_logger().warn("*********************************************")
            self.get_logger().warn("*********************************************")
            self.get_logger().warn("*********************************************")
            self.get_logger().error("BE CAREFUL : YOU HAVE TO COMMENT THIS ON REAL DRONE OR IT WILL CRASH (on sim it is ok)")
            self.get_logger().warn("*********************************************")
            self.get_logger().warn("*********************************************")
            self.get_logger().warn("*********************************************")
        self.get_logger().info(" > FSM started.")
        ### Starting the FSM
        sm = fsm.PopeyeFSM(self)
        ### Save the FSM graph and destry the timer
        img_path = "/home/step/ros2_DUAV/src/popeye/popeye//POPEYE_FSM.png"
        sm._graph().write_png(img_path)
        self.get_logger().warn(" > FSM ended.")
        self.destroy_timer(self.timer__fsm)
    
    ############################################################################################################################################################################################################################
    ##### SUBSCRIBERS CALLBACKS ############################################################################################################################################################################################################################
    def sub_cb__fire(self, msg):
        if msg.is_fire and not self.is_fire:
            self.is_fire  = True
            self.lat_fire = msg.lat_fire
            self.lon_fire = msg.lon_fire
            print()
            self.get_logger().warn(" >>> FIRE HAS BEEN SPOTTED <<<")
            self.get_logger().warn(f" >>> FIRE_LAT:{self.lat_fire} FIRE_LON:{self.lon_fire}<<<")
            print()
    
    ############################################################################################################################################################################################################################
    ##### ACTIONS CLIENTS ############################################################################################################################################################################################################################
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the TAKEOFF action  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__takeoff(self, alt=DEFAULT_ALT):
        self.get_logger().info(f"> Calling TAKEOFF action (alt:{alt})")
        if not self.cli_act__takeoff.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("       ... TAKEOFF action server not available")
            return False
        self.get_logger().info("       ... TAKEOFF action server available")
            
        goal__takeoff = Takeoff.Goal()
        goal__takeoff.alt = float(alt)
        goal_handle_future = self.cli_act__takeoff.send_goal_async(goal__takeoff, feedback_callback=lambda feedback_msg: 
                                                                   self.get_logger().info(f"       ... Feedback (current_alt:{feedback_msg.feedback.current_alt:.1f}, state:{feedback_msg.feedback.state})"))
        rclpy.spin_until_future_complete(self, goal_handle_future)
        if not goal_handle_future.result().accepted:
            self.get_logger().warn("       ... TAKEOFF goal rejected")
            return False
        self.get_logger().info("       ... TAKEOFF goal accepted")
        
        action_future = goal_handle_future.result().get_result_async()
        while not action_future.done():
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.cancel_action:
                cancel_future =  goal_handle_future.result().cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=0.5)
                self.get_logger().warning("       ==> TAKEOFF canceled")
                self.call__rtl()
                return False
            
        if not action_future.result().result.success:
            self.get_logger().warning("       ==> TAKEOFF failed")
            return False
        self.get_logger().info("       ==> TAKEOFF successful")
        self.takeoff_results = True
        return True
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the REPOSITION action  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__reposition(self, lat=DEFAULT_LAT, lon=DEFAULT_LON, alt=DEFAULT_ALT):
        self.get_logger().info(f"> Calling REPOSITION action (lat:{lat}, lon:{lon}, alt:{alt})")
        if not self.cli_act__reposition.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("       ... REPOSITION action server not available")
            return False
        self.get_logger().info("       ... REPOSITION action server available")
            
        goal__reposition = Reposition.Goal()
        goal__reposition.lat = float(lat)
        goal__reposition.lon = float(lon)
        goal__reposition.alt = float(alt)
        goal_handle_future = self.cli_act__reposition.send_goal_async(goal__reposition, feedback_callback=lambda feedback_msg: 
                                                                      self.get_logger().info(f"       ... Feedback (dist_to_goal:{feedback_msg.feedback.dist_to_goal}, delta_alt:{feedback_msg.feedback.delta_alt:.1f}, time_at_position:{feedback_msg.feedback.time_at_position:.1f})"))
        rclpy.spin_until_future_complete(self, goal_handle_future)
        if not goal_handle_future.result().accepted:
            self.get_logger().warn("       ... REPOSITION goal rejected")
            return False
        self.get_logger().info("       ... REPOSITION goal accepted")
        
        action_future = goal_handle_future.result().get_result_async()
        while not action_future.done():
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.cancel_action:
                cancel_future =  goal_handle_future.result().cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=0.5)
                self.get_logger().warning("       ==> REPOSITION canceled")
                self.call__rtl()
                return False
            
        rclpy.spin_until_future_complete(self, action_future)
        if not action_future.result().result.success:
            self.get_logger().warning("       ==> REPOSITION failed")
            return False
        self.get_logger().info("       ==> REPOSITION successful")
        return True
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the LAND action  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__land(self):
        self.get_logger().info(f"> Calling LAND action")
        if not self.cli_act__land.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("       ... LAND action server not available")
            return False
        self.get_logger().info("       ... LAND action server available")
            
        goal__land = Land.Goal()
        goal_handle_future = self.cli_act__land.send_goal_async(goal__land, feedback_callback=lambda feedback_msg: 
                                                                   self.get_logger().info(f"       ... Feedback (Current_alt:{feedback_msg.feedback.current_alt:.1f}, State:{feedback_msg.feedback.state})"))
        rclpy.spin_until_future_complete(self, goal_handle_future)
        if not goal_handle_future.result().accepted:
            self.get_logger().warn("       ... LAND goal rejected")
            return False
        self.get_logger().info("       ... LAND goal accepted")
        
        action_future = goal_handle_future.result().get_result_async()
        rclpy.spin_until_future_complete(self, action_future)
        if not action_future.result().result.success:
            self.get_logger().warning("       ==> LAND failed")
            return False
        self.get_logger().info("       ==> LAND successful")
        return True
    
    ############################################################################################################################################################################################################################
    ##### SERVICES CLIENTS ############################################################################################################################################################################################################################
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the SET_MODE service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__set_mode(self, mode='RTL'):
        self.get_logger().info(f"> Calling SET_MODE (mode:{mode})")
        if not self.cli_srv__set_mode.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning('       ... Service not availabl.')
            return False
        self.get_logger().info("       ... SET_MODE service available")
            
        request           = SetMode.Request()
        request.mode_name = mode
        future            = self.cli_srv__set_mode.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the ARM service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__arm(self, force=False):
        self.get_logger().info(f"> Calling ARM (force:{force})")
        if not self.cli_srv__arm.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning('       ... Service not available.')
            return False
        self.get_logger().info("       ... ARM service available")
        
        request       = Arm.Request()
        request.force = force
        future        = self.cli_srv__arm.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the DROP service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__drop(self):
        self.get_logger().info(f"> Calling DROP ")
        if not self.cli_srv__drop.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning('       ... Service not available.')
            return False
        self.get_logger().info("       ... DROP service available")
        
        request = Drop.Request()
        future  = self.cli_srv__drop.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the RTL service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__rtl(self):
        self.get_logger().info("> Calling RTL.")
        if not self.cli_srv__rtl.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning('       ... Service not available.')
            return False
        self.get_logger().info("       ... RTL service available")
        
        request = Rtl.Request()
        future  = self.cli_srv__rtl.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the DISARM service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__disarm(self, force=False):
        self.get_logger().info(f"> Calling DISARM (force:{force})")
        if not self.cli_srv__disarm.wait_for_service(timeout_sec=3.0):
            self.get_logger().warning('       ... Service not available.')
            return False
        self.get_logger().info("       ... DISARM service available")
        
        self.req__disarm     = Disarm.Request() 
        self.req__disarm.force = force
        future = self.cli_srv__disarm.call_async(self.req__disarm)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
        
#########################################################################################################################################################################################################
##### Node entry point #####################################################################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    
    ### Creating the mutlithread executor
    node = FSMInterfaceNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=5)
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