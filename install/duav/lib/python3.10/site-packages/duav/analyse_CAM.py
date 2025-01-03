#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AnalyseCAM(Node):
    def __init__(self):
        super().__init__('analyse_CAM')
        
        self.publisher_1 = self.create_publisher(String, 'IN/CAM/is_confirmation_needed', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 second interval
        self.get_logger().info("Publisher Node Started")
        
        self.publisher_2 = self.create_publisher(String, 'IN/CAM/is_fire', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 second interval
        self.get_logger().info("Publisher Node Started")
        
        self.publisher_3 = self.create_publisher(String, 'IN/CAM/hot_spot_coord', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 second interval
        self.get_logger().info("Publisher Node Started")
        
    def timer_callback(self):
        msg = String()
        msg.data = "MAV Status: Active"
        self.publisher_1.publish(msg)
        self.get_logger().info(f"Publishing: '{msg.data}'")


def main(args=None):
    rclpy.init(args=args)
    node = AnalyseCAM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
