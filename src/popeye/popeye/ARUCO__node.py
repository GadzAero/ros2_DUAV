#!/usr/bin/env python3

# Import standard utils
import numpy as np 
import math 
from popeye.PARAMS_utils import *
# ROS2 utils
import rclpy 
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# OpenCV utils
from interfaces.msg import GpsPosition
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2
 
#####################################################################################################################################################################
##### Node MAVLink Manager #####################################################################################################################################################################
class ARUCONode(Node):
  def __init__(self):
    super().__init__('ARUCO_node', namespace='POPEYE')
    
    ### SUBSCRIBERS
    self.sub__image_raw = self.create_subscription(Image, 'image_raw', self.sub_cb__image_raw, 10, callback_group=MutuallyExclusiveCallbackGroup())
    
    ### PUBLISHER
    self.pub__cam_fire_pos = self.create_publisher(GpsPosition, 'CAM/fire_pos', 10, callback_group=MutuallyExclusiveCallbackGroup())
    self.pub__cam_park_pos = self.create_publisher(GpsPosition, 'CAM/park_pos', 10, callback_group=MutuallyExclusiveCallbackGroup())
    
    ### GLOBAL PARAMS
    self.cv_bridge = CvBridge()
    fov = math.radians(100)
    self.img_center = np.array([1280/2, 720/2])
    self.constant_pixel_to_meters = 2*math.tan(fov/2) / 1280 / 1.27
   
  ############################################################################################################################################################################################################################
  ##### SUBSCRIBERS CALLBACKS ############################################################################################################################################################################################################################
  #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  #----- Subscriber for VIDEO FRAMES  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  def sub_cb__image_raw(self, msg):
    # self.get_logger().info('Receiving video frame')
    frame = self.cv_bridge.imgmsg_to_cv2(msg)
    # print(self.constant_pixel_to_meters)
    
    if frame is None:
      self.get_logger().warn(" > No images received.")
      return
    
    ## Determine target centers
    aruco_centers = self.find_aruco_centers(frame)
    
    ## Comput offsets
    if aruco_centers is not None:
      for aruco_center in aruco_centers:
        id = aruco_center[0]
        center = aruco_center[1]
        
        ## Compute offset
        offset = self.offset_to_meters(2.5, (center-self.img_center))
        dist_to_target = np.linalg.norm(offset)
        ## For debuging
        if debug_cams:
          self.get_logger().info(f"Offset in px:{center-self.img_center}")
          self.get_logger().info(f"Offset in meters:{offset} => dist:{dist_to_target}")
          cv2.line(frame, (int(self.img_center[0]), int(self.img_center[1])), (int(center[0]), int(center[1])), (0, 255, 0), 1)
        
        ## To publish target poses
        if id == 5:
          msg_pub     = GpsPosition()
          msg_pub.lat = float(offset[0])
          msg_pub.lon = float(offset[1])
          msg_pub.alt = float(0.)
          self.pub__cam_park_pos.publish(msg_pub)
        elif id == 222:
          msg_pub     = GpsPosition()
          msg_pub.lat = float(offset[0])
          msg_pub.lon = float(offset[1])
          msg_pub.alt = float(0.)
          self.pub__cam_fire_pos.publish(msg_pub)
        else:
          print("This ARUCO tag is not used")
    
    if debug_cams:
      cv2.imshow("camera", frame)
      cv2.waitKey(1)
  
  ############################################################################################################################################################################################################################
  ##### TOOLS ############################################################################################################################################################################################################################
  #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  #----- Converts an offset in pixels in offset in meters using camera specifications and drone altitude------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  def offset_to_meters(self, altitude, offset):
      ## Insert camera specifications for FOV 
      """OV2710 (format 1/2.7", dimensions 5856µm x 3276µm) avec différents objectifs possibles — par exemple un CS 5-50 mm pour le modèle ELP-USBFHD01M-SFV(5-50)."""
      ## OV2710 FOV at 5 mm focal (max unzoom)
      offset_m = np.array(offset)*altitude*self.constant_pixel_to_meters                       
      return offset_m 
  #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  #----- Fonction to FIND ARUCO CENTERS ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  def find_aruco_centers(self, frame):
    ## Select the needed parameters
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
    parameters =  cv2.aruco.DetectorParameters()
    
    ## Find the aruco markers
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Enhance detection
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    marker_corners, marker_ids, _ = detector.detectMarkers(gray)
    if marker_ids is None:
      return
    
    ## Get the center
    aruco_centers = []
    for i, id in enumerate(marker_ids):
      corner = marker_corners[i][0]
      center_x = int((corner[0][0] + corner[2][0]) / 2)
      center_y = int((corner[0][1] + corner[2][1]) / 2)
      aruco_centers.append([id, np.array([center_x, center_y])])
      if debug_cams:
        cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
    
    if debug_cams:
      cv2.aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)
      
    return aruco_centers
  
#########################################################################################################################################################################################################
##### Node entry point #####################################################################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    
    ### Creating the mutlithread executor
    node = ARUCONode()
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