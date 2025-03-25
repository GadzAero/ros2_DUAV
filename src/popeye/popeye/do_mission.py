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
from popeye.read_pos import *

import time
import rclpy
from rclpy.node import Node
        
##############################################
##### Node related to the mission orders #####
class DoMission(Node):
    ### Class constructor ###
    def __init__(self):
        super().__init__('do_mission', namespace='POPEYE')
        
        #-- SERVICES --#
        # Prepare set_mode client
        self.cli__set_mode  = self.create_client(SetMode, '/mavros/set_mode')
        for attempt in range(5):
            if self.cli__set_mode.wait_for_service(timeout_sec=1.0):
                break
            self.get_logger().info('Service /mavros/set_mode not available, waiting again...')
        self.req__set_mode = SetMode.Request()
        # Prepare arming client
        self.cli__trigger_arm  = self.create_client(CommandBool, '/mavros/cmd/arming')
        for attempt in range(5):
            if self.cli__trigger_arm.wait_for_service(timeout_sec=1.0):
                break
            self.get_logger().info('Service /mavros/cmd/arming not available, waiting again...')
        self.req__trigger_arm = CommandBool.Request()
        # Prepare takeoff client
        self.cli__trigger_takeoff  = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        for attempt in range(5):
            if self.cli__trigger_takeoff.wait_for_service(timeout_sec=1.0):
                break
            self.get_logger().info('Service /mavros/cmd/takeoff not available, waiting again...')
        self.req__trigger_takeoff = CommandTOL.Request()
        # Prepare get_av_pose client
        self.cli__get_uav_pose  = self.create_client(GetPoseAAA, '/POPEYE/get_pose')
        for attempt in range(5):
            if self.cli__get_uav_pose.wait_for_service(timeout_sec=1.0):
                break
            self.get_logger().info('Service /POPEYE/get_pose not available, waiting again...')
        self.req__get_uav_pose = GetPoseAAA.Request()

        #-- TOPICS --#
        # Publishers
        self.pub__move_to_pos   = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.pub__move_to_speed = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

        self.get_logger().info("NODE do_mission STARTED.")
    
    ### Function to CHANGE_MODE ###
    def change_mode(self, custom_mode='GUIDED'):
        self.req__set_mode.custom_mode = custom_mode
        return self.cli__set_mode.call_async(self.req__set_mode)
    
    ### Function to ARM  ###
    def trigger_arm(self, arm=True):
        self.req__trigger_arm.value = arm
        return self.cli__trigger_arm.call_async(self.req__trigger_arm)
    
    ### Function to TAKEOFF ###
    def trigger_takeoff(self, altitude=10):
        self.req__trigger_takeoff.altitude = altitude
        return self.cli__trigger_takeoff.call_async(self.req__trigger_takeoff)
    
    ### Function to MOVE_TO_POS ###
    def move_to_pos(self, north=0, east=0, up=0, yaw=0, debug=False):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.pose.position.x = north
        msg.pose.position.y = east
        msg.pose.position.z = up
        rotation = Rotation.from_euler('zyx', [yaw/180*3.14159, 0, 0])
        quaternion = rotation.as_quat()
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]
# A CORRIGER !!!!
        for i in range(100000000):
            self.pub__move_to_pos.publish(msg)
            time.sleep(0.1)
        if debug:
            self.get_logger().info('Publishing on /mavros/setpoint_position/local : \'east:%s north:%s up:%s yaw:%s\'' % (str(east), str(north), str(up), str(yaw)))
       
    ### Function to MOVE_TO_SPEED ### 
    def move_to_speed(self, x=0, y=0, z=0, yaw=0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.twist.linear.x = x
        msg.twist.linear.y = y
        msg.twist.linear.z = z
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = yaw/180*3.14159
        self.pub__move_to_speed.publish(msg)
        
    ### Function to RELATIVE_MOVE_TO_POSE ### 
    def relative_move_to_pose(self, north=0, east=0, up=0, yaw=0, debug=True):
        # Call the service to get the current UAV pose
        future = self.cli__get_uav_pose.call_async(self.req__get_uav_pose)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        current_pose = response.pose
        # Compute next pose and move the UAV
        relative_east = current_pose.pose.position.x + east
        relative_north = current_pose.pose.position.y + north
        relative_up = current_pose.pose.position.z + up
        relative_yaw = 0 #current_pose.pose.orientation.yaw + yaw
        self.move_to_pos(relative_east, relative_north, relative_up, 0, False)
        if debug:
            self.get_logger().info('Moving relatively to current_pose : \'east:%s north:%s up:%s yaw:%s\'' % (str(east), str(north), str(up), str(yaw)))
            

##############################
##### Nodes entry points #####
def main(args=None):
    rclpy.init(args=args)
    
    node = DoMission()
    
    # Change Mode
    # future = node.change_mode(str(sys.argv[1]))
    # rclpy.spin_until_future_complete(node, future)
    # response = future.result()
    # node.get_logger().info('Result of change_mode to \'%s\' : %s' % (str(sys.argv[1]), str(response.mode_sent)))
    
    # # Arm UAV
    # future = node.trigger_arm(bool(sys.argv[2]))
    # rclpy.spin_until_future_complete(node, future)
    # response = future.result()
    # node.get_logger().info('Result of trigger_arm to \'%s\' : %s -> %s ' % (str(sys.argv[2]), str(response.success), str(response.result)))
    
    # # Takeoff
    # future = node.trigger_takeoff(float(sys.argv[3]))
    # rclpy.spin_until_future_complete(node, future)
    # response = future.result()
    # node.get_logger().info('Result of trigger_takeoff to \'%s\' : %s -> %s' % (str(sys.argv[3]), str(response.success), str(response.result)))
    
    # # Wait for UAV to takeoff
    # delay_seconds = 20 
    # node.get_logger().info(f'Waiting TAKEOFF for {delay_seconds} seconds...')
    # time.sleep(delay_seconds)
    
    # # Go to point
    # node.move_to_pos(float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7]))
    
    # Go to relative point from current_pose
    node.relative_move_to_pose(float(sys.argv[8]), float(sys.argv[9]), float(sys.argv[10]), float(sys.argv[11]))
    
    # Go to speed
    # for i in range(10000):
    #     node.move_to_speed(float(sys.argv[8]), float(sys.argv[9]), float(sys.argv[10]), float(sys.argv[11]))
    #     # rclpy.spin_once(node)
    #     node.get_logger().info('Publishing on /mavros/setpoint_velocity/cmd_vel : \'east:%s north:%s up:%s yaw:%s\'' % (str(sys.argv[8]), str(sys.argv[9]), str(sys.argv[10]), str(sys.argv[11])))

    # # Wait for UAV to go to location
    # delay_seconds = 20 
    # node.get_logger().info(f'Waiting GO TO for {delay_seconds} seconds...')
    # time.sleep(delay_seconds)
    
    # Change Mode
    # future = node.change_mode('RTL')
    # rclpy.spin_until_future_complete(node, future)
    # response = future.result()
    # node.get_logger().info('Result of change_mode to \'RTL\' : %s' % (str(response.mode_sent)))

    node.destroy_node()
    rclpy.shutdown()