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
class CAMNode(Node):
  def __init__(self):
    super().__init__('CAM_node', namespace='POPEYE')
    
    ### ROS2 callbacks
    ## Publishers
    self.pub__video_frames = self.create_publisher(Image, 'image_raw', 10, callback_group=MutuallyExclusiveCallbackGroup())
    ## Timers
    self.timer__read_frames = self.create_timer(0.05, self.timer_cb__read_frames, callback_group=MutuallyExclusiveCallbackGroup())
    
    ### START VIDEO CAPTURE
    # self.vd_capture = cv2.VideoCapture(path_DUAV+"/src/popeye/popeye/videos/test_offset.mp4")
    # self.vd_capture = cv2.VideoCapture(path_DUAV+"/src/popeye/popeye/videos/test_offset_diag.mp4")
    # self.vd_capture = cv2.VideoCapture(path_DUAV+"/src/popeye/popeye/videos/precision_landing.mp4")
    self.vd_capture = cv2.VideoCapture(path_DUAV+"/src/popeye/popeye/videos/precision_landing_straight.mp4")
    self.cv_bridge = CvBridge()
   
  ############################################################################################################################################################################################################################
  ##### TIMER CALLBACK ############################################################################################################################################################################################################################
  #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  #----- Function to publish VIDEO FRAMES  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  def timer_cb__read_frames(self):
    capture_success, frame = self.vd_capture.read()
    if capture_success is not None:
      self.pub__video_frames.publish(self.cv_bridge.cv2_to_imgmsg(frame))
 
    self.get_logger().info('Publishing video frame')
  
#########################################################################################################################################################################################################
##### Node entry point #####################################################################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    
    ### Creating the mutlithread executor
    node = CAMNode()
    executor = rclpy.executors.SingleThreadedExecutor()
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