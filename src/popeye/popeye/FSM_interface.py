#!/usr/bin/env python3

# Import ROS2 utils
import rclpy
from rclpy.node import Node

# Import FSM utils
# import popeye.utils.FSM_utils as fsm

#####################################################################################################################################################################
##### Node MAVLink Manager #####################################################################################################################################################################
class FSMInterface(Node):
    def __init__(self):
        super().__init__('FSM_interface', namespace='POPEYE')
        
        self.get_logger().info("NODE FSM_interface STARTED.")
        
    
        
        
#####################################################################################################################################################################
##### Node entry point #####################################################################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    node = FSMInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()