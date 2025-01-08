#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SelectMissionType(Node):
    def __init__(self):
        super().__init__('select_mission_type')
        
        self.publisher_ = self.create_publisher(String, 'INTERN/mission_type', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 second interval
        self.get_logger().info("Publisher Node Started")
        
        self.subscription_1 = self.create_subscription(String,'IN/CAM/is_confirmation_needed',self.listener_callback_1,10)
        self.subscription_1
        
        self.subscription_2= self.create_subscription(String,'IN/CAM/is_fire',self.listener_callback_1,10)
        self.subscription_2
        
        self.subscription_3 = self.create_subscription(String,'IN/CAM/hot_spot_coord',self.listener_callback_1,10)
        self.subscription_3
        
        self.subscription_4 = self.create_subscription(String,'INTERN/plane_state',self.listener_callback_1,10)
        self.subscription_4
        
    def timer_callback(self):
        msg = String()
        msg.data = "MAV Status: Active"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: '{msg.data}'")
        
    def listener_callback_1(self, msg):
        self.get_logger().info(f"Received: '{msg.data}'")


def main(args=None):
    rclpy.init(args=args)
    node = SelectMissionType()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
