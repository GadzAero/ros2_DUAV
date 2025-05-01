#!/usr/bin/env python3

# Import standard utils
from popeye.PARAMS_utils import *
# ROS2 utils
import rclpy 
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# OpenCV utils
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2
 
#####################################################################################################################################################################
##### Node MAVLink Manager #####################################################################################################################################################################
class CAMAnalyser(Node):
  def __init__(self):
    super().__init__('CAM_aruco', namespace='POPEYE')
    
    ### SUBSCRIBERS
    self.sub__video_frames = self.create_subscription(Image, 'video_frames', self.sub_cb__video_frames, 10)
    
    ### GLOBAL PARAMS
    self.cv_bridge = CvBridge()
   
  ############################################################################################################################################################################################################################
  ##### SUBSCRIBERS CALLBACKS ############################################################################################################################################################################################################################
  #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  #----- Subscriber for VIDEO FRAMES  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  def sub_cb__video_frames(self, msg):
    self.get_logger().info('Receiving video frame')
    current_frame = self.cv_bridge.imgmsg_to_cv2(msg)
    if not on_raspi:
      cv2.imshow("camera", current_frame)
    cv2.waitKey(1)
  
#########################################################################################################################################################################################################
##### Node entry point #####################################################################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    
    ### Creating the mutlithread executor
    node = CAMAnalyser()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=5)
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