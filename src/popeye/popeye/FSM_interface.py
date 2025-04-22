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
from interfaces.srv import SetMode, Arm, Rtl, Disarm
from interfaces.action import Takeoff, Land, Reposition
# Import FSM utils
import popeye.utils_FSM as fsm
from popeye.utils_MAV import DEFAULT_LAT, DEFAULT_LON, DEFAULT_ALT

import asyncio

#####################################################################################################################################################################
##### Node MAVLink Manager #####################################################################################################################################################################
class FSMInterface(Node):
    def __init__(self):
        super().__init__('FSM_interface', namespace='POPEYE')
        self.get_logger().info("NODE FSM_interface STARTED.")
        
        ### Services clients
        self.cli_srv__set_mode   = self.create_client(SetMode,    'set_mode',   callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_srv__arm        = self.create_client(Arm,        'arm',        callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_srv__rtl        = self.create_client(Rtl,        'rtl',        callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_srv__disarm     = self.create_client(Disarm,     'disarm',     callback_group=MutuallyExclusiveCallbackGroup())
        while (not self.cli_srv__set_mode.      wait_for_service(timeout_sec=1.0)
                or not self.cli_srv__arm.       wait_for_service(timeout_sec=1.0)
                # or not self.cli_srv__reposition.wait_for_service(timeout_sec=1.0)
                or not self.cli_srv__rtl.       wait_for_service(timeout_sec=1.0)
                or not self.cli_srv__disarm.    wait_for_service(timeout_sec=1.0)):
            self.get_logger().warning('Service(s) not available, waiting again...')
        self.req__set_mode = SetMode.Request()
        self.req__arm        = Arm.Request()
        # self.req__reposition = Reposition.Request()
        self.req__rtl        = Rtl.Request()
        self.req__disarm     = Disarm.Request() 
        
        ### Actions clients
        self.cli_act__takeoff    = ActionClient(self, Takeoff,    'takeoff',    callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_act__land       = ActionClient(self, Land,       'land',       callback_group=MutuallyExclusiveCallbackGroup())
        self.cli_act__reposition = ActionClient(self, Reposition, 'reposition', callback_group=MutuallyExclusiveCallbackGroup())
        
        ### Global prarams
        self.cancel_action = False
        
        ### Start the FSM
        self.call__disarm(force=True)
        self.get_logger().warn("BE CAREFUL : YOU HAVE TO COMMENT THIS ON REAL DRONE OR IT WILL CRASH")
        sm = fsm.PopeyeFSM(self)
        img_path = "/home/step/ros2_DUAV/src/popeye/popeye//POPEYE_FSM.png"
        sm._graph().write_png(img_path)
        # self.result = None
        # self.call__set_mode(mode='GUIDED')
        # self.call__arm()
        # self.goal_handle_future = self.call__takeoff()
        # print("okkkk")
        # while rclpy.ok():
        #     rclpy.spin_once(self, timeout_sec=0.1)
    
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
    # def call__takeoff(self, alt=DEFAULT_ALT):
    #     self.get_logger().info(f"> Calling TAKEOFF action (alt:{alt})")

    #     if not self.cli_act__takeoff.wait_for_server(timeout_sec=3.0):
    #         self.get_logger().warn("       ... TAKEOFF action server not available")
    #         return False
    #     self.get_logger().info("       ... TAKEOFF action server available")

    #     goal__takeoff = Takeoff.Goal()
    #     goal__takeoff.alt = float(alt)
    #     self.feedback_callback  = lambda feedback_msg: self.get_logger().info(f"       ... Feedback (current_alt:{feedback_msg.feedback.current_alt:.1f}, state:{feedback_msg.feedback.state})")
    #     self.goal_handle_future = self.cli_act__takeoff.send_goal_async(goal__takeoff, feedback_callback=self.feedback_callback)
    #     self.goal_handle_future.add_done_callback(self.takeoff__handle_goal_response)

    #     return self.goal_handle_future
    # def takeoff__handle_goal_response(self, future):
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().warn("       ... TAKEOFF goal rejected")
    #         return
    #     self.get_logger().info("       ... TAKEOFF goal accepted")
    #     goal_handle.get_result_async().add_done_callback(self.takeoff__handle_takeoff_result)
    # def takeoff__handle_takeoff_result(self, future):
    #     self.result = future.result().result
    #     if not self.result.success:
    #         self.get_logger().warning("       ==> TAKEOFF failed")
    #         return
    #     self.get_logger().info("       ==> TAKEOFF successful")
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
        self.req__set_mode.mode_name = mode
        self.get_logger().info(f"> Calling SET_MODE (Force:{self.req__set_mode.mode_name})")
        while not self.cli_srv__set_mode.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning('Service SET_MODE not available, waiting again...')
        future = self.cli_srv__set_mode.call_async(self.req__set_mode)
        
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the ARM service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__arm(self, force=False):
        self.req__arm.force = force
        self.get_logger().info(f"> Calling ARM (Force:{self.req__arm.force})")
        future = self.cli_srv__arm.call_async(self.req__arm)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
            
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the RTL service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__rtl(self):
        self.get_logger().info("> Calling RTL.")
        future = self.cli_srv__rtl.call_async(self.req__rtl)
        rclpy.spin_until_future_complete(self, future)
        # sleep(30)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
            
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to call the DISARM service  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def call__disarm(self, force=False):
        self.req__disarm.force = force
        self.get_logger().info(f"> Calling DISARM (Force:{self.req__disarm.force})")
        future = self.cli_srv__disarm.call_async(self.req__disarm)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"     -> Successful")
        else:
            self.get_logger().warning(f"     -> Failed")
        
#####################################################################################################################################################################
##### Node entry point #####################################################################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    
    ### Creating the mutlithread executor
    node = FSMInterface()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=5)
    # executor = rclpy.executors.SingleThreadedExecutor()
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