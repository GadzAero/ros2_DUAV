# pub_MAV.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class WriteMAV(Node):
    def __init__(self):
        super().__init__('write_MAV')
        
        self.subscription_1 = self.create_subscription(String,'OUT/path',self.listener_callback_1,10)
        self.subscription_1
        
        self.subscription_3 = self.create_subscription(String,'IN/CAM/is_fire',self.listener_callback_1,10)
        self.subscription_3
        
    def listener_callback_1(self, msg):
        self.get_logger().info(f"Received: '{msg.data}'")

def main(args=None):
    rclpy.init(args=args)
    node =WriteMAV()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
