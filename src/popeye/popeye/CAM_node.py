#!/usr/bin/env python3

# Import standard utils
from time import sleep
# Import ROS2 utils
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# Import Intefaces
from interfaces.srv import SetMode, Arm, Rtl, Disarm, Drop
from interfaces.action import Takeoff, Land, Reposition
from interfaces.msg import Fire, UavPosition, UavAttitude

#####################################################################################################################################################################
##### Node for camera use #####################################################################################################################################################################
class CAMNode(Node):
    def __init__(self):
        super().__init__('CAM_node', namespace='POPEYE')
        
        ### SERVICE CLIENTS
        self.cli_srv__set_mode = self.create_client(SetMode, 'set_mode', callback_group=MutuallyExclusiveCallbackGroup())
        
        ### SUBSCRIBERS 
        self.sub__position = self.create_subscription(UavPosition, 'position', self.sub_cb__position, 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.sub__attitude = self.create_subscription(UavAttitude, 'attitude', self.sub_cb__position, 10, callback_group=MutuallyExclusiveCallbackGroup())
        
    ############################################################################################################################################################################################################################
    ##### TIMERS CALLBACKS ############################################################################################################################################################################################################################
    def timer_cb__fire_search(self):
        pass
    
    ############################################################################################################################################################################################################################
    ##### SUBSCRIBERS CALLBACKS ############################################################################################################################################################################################################################
    def sub_cb__position(self, msg):
        self.lat = msg.lat
        self.lon = msg.lon
        self.alt = msg.alt
    
    def sub_cb__attitude(self, msg):
        self.yaw   = msg.yaw

#########################################################################################################################################################################################################
##### Node entry point #####################################################################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    
    ### Creating the mutlithread executor
    node = CAMNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()