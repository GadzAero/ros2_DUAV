#!/usr/bin/env python3

# Import standard utils
from popeye.PARAMS_utils import *
from time import sleep
# ROS2 utils
import rclpy 
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# Interfaces
from interfaces.msg import Fire, GpsPosition
from interfaces.action import Takeoff, Land, Reposition
 
#####################################################################################################################################################################
##### Node MAVLink Manager #####################################################################################################################################################################
class ASSERVNode(Node):
    def __init__(self):
        super().__init__('ASSERV_node', namespace='POPEYE')
        
        ### ACTION SERVERS
        ActionServer(self, PrecisionLand, 'precision_land', self.act_cb__precision_land, callback_group=MutuallyExclusiveCallbackGroup())
        
        ### SUBSCRIBERS
        self.sub__uav_position = self.create_subscription(GpsPosition, 'uav_position', self.sub_cb__uav_position, 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.sub__cam_fire_pos = self.create_subscription(GpsPosition, 'CAM/fire_pos', self.sub_cb__cam_fire_pos, 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.sub__cam_park_pos = self.create_subscription(GpsPosition, 'CAM/park_pos', self.sub_cb__cam_park_pos, 10, callback_group=MutuallyExclusiveCallbackGroup())

    ############################################################################################################################################################################################################################
    ##### SUBSCRIBERS CALLBACKS ############################################################################################################################################################################################################################
    # #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # #----- Subscriber for UAV_POSITION  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # def sub_cb__uav_position(self, msg):
    #     print("uav_pos")
    #     self.uav_pos = (msg.lat, msg.lon)
    #     self.uav_alt = msg.alt
    # #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # #----- Subscriber for CAM FIRE POS  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # def sub_cb__cam_fire_pos(self, msg):
    #     print("cam_pos")
    #     self.pos_cam_fire = (msg.lat, msg.lon)
    #     self.alt_cam_fire = msg.alt
    #     print()
    #     # self.get_logger().info(f" >>> CAM_FIRE_POS:{self.pos_cam_fire} <<<")
    #     # print()
    # #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # #----- Subscriber for CAM PARK POS  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # def sub_cb__cam_park_pos(self, msg):
    #     print("park_pos")
    #     self.pos_cam_park = (msg.lat, msg.lon)
    #     self.alt_cam_park = msg.alt
    #     print()
    #     # self.get_logger().info(f" >>> CAM_PARK_POS:{self.pos_cam_park} <<<")
    #     # print()
    
############################################################################################################################################################################################################################
##### ACTIONS CALLBACK ############################################################################################################################################################################################################################
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#----- Action server to REPOSITION ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# def act_cb__precision_land(self, goal_handle):
#     print()
#     self.get_logger().info(f"> Call action REPOSITION (lat:{goal_handle.request.lat}, lon:{goal_handle.request.lon}, alt:{goal_handle.request.alt})")
    
#     if not mav_utils.mav_reposition(self.mav_master, goal_handle.request.lat, goal_handle.request.lon, goal_handle.request.alt):
#         self.get_logger().warning("      -> Failure: command not valid")
#         goal_handle.abort()
#         return Reposition.Result(success=False)
    
#     time_at_position = 0; sleep_time = 1
#     tolerance = 1.0e-06
#     while time_at_position < 5:
#         dist_to_goal = ((goal_handle.request.lat-self.popeye_pos_lat)**2 + (goal_handle.request.lon-self.popeye_pos_lon)**2)**0.5
#         delta_alt    = goal_handle.request.alt-self.popeye_pos_alt
#         time_at_position = time_at_position+sleep_time if (dist_to_goal<=tolerance and delta_alt<0.15) else 0
#         goal_handle.publish_feedback(Reposition.Feedback(time_at_position=float(time_at_position), dist_to_goal=dist_to_goal, delta_alt=delta_alt))
#         self.get_logger().info(f"      ... Repositioning (dist_to_goal:{dist_to_goal}, delta_alt:{delta_alt:.1f}, time_at_position:{time_at_position})")
#         sleep(sleep_time)
#     self.get_logger().info("      -> Success")
#     goal_handle.succeed()
#     return Reposition.Result(success=True)
        
    
   
  
#########################################################################################################################################################################################################
##### Node entry point #####################################################################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    
    ### Creating the mutlithread executor
    node = ASSERVNode()
    executor = rclpy.executors.SingleThreadedExecutor()
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