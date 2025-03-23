#!/usr/bin/env python3

import sys
import os
sys.path.append('~/ros2_DUAV/src/mavros')
# sys.path.append(os.path.join(os.path.dirname(__file__), '../../mavros'))
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from geometry_msgs.msg import PoseStamped, TwistStamped
from scipy.spatial.transform import Rotation
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

import time
import rclpy
from rclpy.node import Node

# Variable for the GPS position
current_pose = PoseStamped()

#############################################################
##### Node to subscribe to the GPS position MAVlink msg #####
class GetPose(Node):
    ### SAHRED VARIABLES ###
    current_pose = PoseStamped()
    test = 1
    def __init__(self):
        super().__init__('get_pose', namespace='POPEYE')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.sub__move_to_speed = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profile)
        
    def pose_callback(self, msg):
        current_pose = msg
        self.get_logger().info("READING CURRENT POSE")
        self.get_logger().info("   >x(m)     :"+str(current_pose.pose.position.x))
        self.get_logger().info("   >y(m)     :"+str(current_pose.pose.position.y))
        self.get_logger().info("   >z(m)     :"+str(current_pose.pose.position.z))
        rotation = Rotation.from_quat([current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w])
        euler_angles = rotation.as_euler('zyx', degrees=True)
        self.get_logger().info("   >yaw(°)   :"+str(euler_angles[0]))
        self.get_logger().info("   >pitch(°) :"+str(euler_angles[1]))
        self.get_logger().info("   >roll(°)  :"+str(euler_angles[2]))
        
##############################################
##### Node related to the mission orders #####
class DoMission(Node):
    def __init__(self):
        super().__init__('do_mission', namespace='POPEYE')
        
        ### SERVICES ###
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
        # Prepare arming client
        self.cli__trigger_takeoff  = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        for attempt in range(5):
            if self.cli__trigger_takeoff.wait_for_service(timeout_sec=1.0):
                break
            self.get_logger().info('Service /mavros/cmd/takeoff not available, waiting again...')
        self.req__trigger_takeoff = CommandTOL.Request()
        
        ### TOPICS ###
        # Publishers
        self.pub__move_to_pos   = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.pub__move_to_speed = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)

        self.get_logger().info("NODE do_mission STARTED.")
        
    def change_mode(self, custom_mode='GUIDED'):
        self.req__set_mode.custom_mode = custom_mode
        return self.cli__set_mode.call_async(self.req__set_mode)
    
    def trigger_arm(self, arm=True):
        self.req__trigger_arm.value = arm
        return self.cli__trigger_arm.call_async(self.req__trigger_arm)
    
    def trigger_takeoff(self, altitude=10):
        self.req__trigger_takeoff.altitude = altitude
        return self.cli__trigger_takeoff.call_async(self.req__trigger_takeoff)
    
    def move_to_pos(self, north=0, east=0, up=0, yaw=0):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.pose.position.x = east 
        msg.pose.position.y = north 
        msg.pose.position.z = up
        rotation = Rotation.from_euler('zyx', [yaw/180*3.14159, 0, 0])
        quaternion = rotation.as_quat()
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]
        
        for i in range(10):
            self.pub__move_to_pos.publish(msg)
            time.sleep(0.1)
        self.get_logger().info('Publishing on /mavros/setpoint_position/local : \'east:%s north:%s up:%s yaw:%s\'' % (str(east), str(north), str(up), str(yaw)))
        
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
        
    def move_relative_to_pose(self, east=0, north=0, up=0, yaw=0):
        print(DoMission.current_pose.pose.position.x)
        print(DoMission.test)
        relative_east = DoMission.current_pose.pose.position.x + east
        relative_north = DoMission.current_pose.pose.position.y + north
        relative_up = DoMission.current_pose.pose.position.z + up
        # relative_east = DoMission.current_pose.pose.position. + yaw
        self.move_to_pos(relative_east, relative_north, relative_up, 0)
        self.get_logger().info('Moving relatively to current_pose : \'east:%s north:%s up:%s yaw:%s\'' % (str(east), str(north), str(up), str(yaw)))
   
def main(args=None):
    rclpy.init(args=args)
    
    node = GetPose()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()