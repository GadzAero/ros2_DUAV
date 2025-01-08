#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CreatePathPubliser(Node):
    def __init__(self):
        super().__init__('create_path')
        
        self.publisher_ = self.create_publisher(String, 'OUT/path', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 second interval
        self.get_logger().info("Publisher Node Started")
        
        
        self.subscription_4 = self.create_subscription(String,'INTERN/costmap',self.listener_callback_1,10)
        self.subscription_4
        
        self.subscription_5 = self.create_subscription(String,'INTERN/mission_type',self.listener_callback_1,10)
        self.subscription_5
        
    def timer_callback(self):
        msg = String()
        msg.data = "MAV Status: Active"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: '{msg.data}'")
        
    def listener_callback_1(self, msg):
        self.get_logger().info(f"Received: '{msg.data}'")

def main(args=None):
    rclpy.init(args=args)
    node = CreatePathPubliser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
