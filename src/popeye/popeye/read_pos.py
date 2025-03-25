#!/usr/bin/env python3

import sys
import os
sys.path.append('~/ros2_DUAV/src/mavros')
# sys.path.append(os.path.join(os.path.dirname(__file__), '../../mavros'))
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from geometry_msgs.msg import PoseStamped, TwistStamped
from scipy.spatial.transform import Rotation
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from interfaces.srv import TriggerArm, GetPoseAAA

import time
import rclpy
from rclpy.node import Node

#############################################################
##### Node to subscribe to the GPS position MAVlink msg #####
class GetUAVPose(Node):
    ### Class constructor ###
    def __init__(self):
        super().__init__('get_pose', namespace='POPEYE')
        
        #-- QoS --#
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        #-- SERVICES --#
        self.srv__uav_pose = self.create_service(GetPoseAAA, 'get_pose', self.sc__uav_pose)
        
        #-- TOPICS --#
        self.sub__move_to_speed = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.tc__pose_callback, qos_profile)
        self.current_pose = PoseStamped()
        
        self.get_logger().info("NODE get_uav_pose STARTED.")
        
    ### Service callback for uav POSE ###
    def sc__uav_pose(self, request, response):
        response.pose = self.current_pose
        response.success = True
        return response
    
    ### Function to CHANGE_MODE ###
    def tc__pose_callback(self, msg):
        self.current_pose = msg
        
##############################
##### Nodes entry points #####
def main(args=None):
    rclpy.init(args=args)

    node = GetUAVPose()
    rclpy.spin(node)
    
    rclpy.shutdown()