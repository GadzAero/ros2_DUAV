#!/usr/bin/env python3

from popeye.do_mission import *

def main(args=None):
    rclpy.init(args=args)
    
    node = DoMission()
    
    # # Change Mode
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
    node.move_relative_to_pose(float(sys.argv[8]), float(sys.argv[9]), float(sys.argv[10]), float(sys.argv[11]))
    
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