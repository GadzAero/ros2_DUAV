# pub_MAV.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MAVPublisher(Node):
    def __init__(self):
        super().__init__('pub_MAV')
        
        self.publisher_1 = self.create_publisher(String, 'IN/MAV/gps_coord', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 second interval
        self.get_logger().info("Publisher Node Started")
        
        self.publisher_2 = self.create_publisher(String, 'IN/MAV/fences', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 second interval
        self.get_logger().info("Publisher Node Started")
        
        self.publisher_3 = self.create_publisher(String, 'IN/MAV/altitude', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 second interval
        self.get_logger().info("Publisher Node Started")
        
        self.publisher_4 = self.create_publisher(String, 'IN/MAV/no_go_zones', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 second interval
        self.get_logger().info("Publisher Node Started")
        
        self.publisher_5 = self.create_publisher(String, 'IN/MAV/imu', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 second interval
        self.get_logger().info("Publisher Node Started")
        
        self.publisher_6 = self.create_publisher(String, 'IN/MAV/attitude', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1 second interval
        self.get_logger().info("Publisher Node Started")

    def timer_callback(self):
        msg = String()
        msg.data = "MAV Status: Active"
        self.publisher_1.publish(msg)
        self.get_logger().info(f"Publishing: '{msg.data}'")

def main(args=None):
    rclpy.init(args=args)
    node = MAVPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
