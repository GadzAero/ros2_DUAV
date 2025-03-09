#!/usr/bin/env python3

import sys
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from geometry_msgs.msg import PoseStamped, TwistStamped
from scipy.spatial.transform import Rotation

import time
import rclpy
from rclpy.node import Node

class DoMission(Node):
    def __init__(self):
        super().__init__('do_mission', namespace='POPEYE')

        timeout_count=0

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
        self.pub__move_to_pos = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
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
        
        self.pub__move_to_pos.publish(msg)
        
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
    
    def asserv_pos(self, x, y, z, yaw):
        
    
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
    # rclpy.spin_once(node)
    # node.get_logger().info('Publishing on /mavros/setpoint_position/local : \'east:%s north:%s up:%s yaw:%s\'' % (str(sys.argv[4]), str(sys.argv[5]), str(sys.argv[6]), str(sys.argv[7])))
    
    # Go to speed
    for i in range(10000):
        node.move_to_speed(float(sys.argv[8]), float(sys.argv[9]), float(sys.argv[10]), float(sys.argv[11]))
        # rclpy.spin_once(node)
        node.get_logger().info('Publishing on /mavros/setpoint_velocity/cmd_vel : \'east:%s north:%s up:%s yaw:%s\'' % (str(sys.argv[8]), str(sys.argv[9]), str(sys.argv[10]), str(sys.argv[11])))

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

if __name__ == '__main__':
    main()