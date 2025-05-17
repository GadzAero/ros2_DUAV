#!/usr/bin/env python3

# Import standard utils
import numpy as np 
import math 
from time import sleep
from popeye.PARAMS_utils import *
# ROS2 utils
import rclpy 
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# OpenCV utils
from interfaces.msg import GpsPosition, Attitude
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2
 
#####################################################################################################################################################################
##### Node MAVLink Manager #####################################################################################################################################################################
class ARUCONode(Node):  
  def __init__(self):
    super().__init__('ARUCO_node', namespace='POPEYE')
    
    ### ROS2 Callbacls
    ## Callback groups
    non_critical_group = ReentrantCallbackGroup()
    ## Subscribers
    self.create_subscription(Image,       'image_raw',    self.sub_cb__image_raw,    10, callback_group=non_critical_group)
    self.create_subscription(GpsPosition, 'uav_position', self.sub_cb__uav_position, 10, callback_group=non_critical_group)
    self.create_subscription(Attitude,    'uav_attitude', self.sub_cb__uav_attitude, 10, callback_group=non_critical_group)
    ## Publishers
    self.pub__cam_fire_pos = self.create_publisher(GpsPosition, 'CAM/fire_pos', 10, callback_group=non_critical_group)
    self.pub__cam_park_pos = self.create_publisher(GpsPosition, 'CAM/park_pos', 10, callback_group=non_critical_group)
    self.pub__img_debug    = self.create_publisher(Image,       'image_debug',  10, callback_group=non_critical_group)
    
    ### GLOBAL PARAMS
    self.cv_bridge = CvBridge()
    fov = math.radians(100)
    self.img_center = np.array([1280/2, 720/2])
    self.constant_pixel_to_meters = 2*math.tan(fov/2) / 1280 / 1.27 # As been corrected: do not remove the last division
    self.uav_position = None
    
   
  ############################################################################################################################################################################################################################
  ##### SUBSCRIBERS CALLBACKS ############################################################################################################################################################################################################################
  #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  #----- Subscriber for UAV_POSITION  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  def sub_cb__uav_position(self, msg):
    self.uav_position = (msg.lat, msg.lon)
    self.uav_alt = msg.alt
  #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  #----- Subscriber for UAV_ATTITUDE  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  def sub_cb__uav_attitude(self, msg):
    self.uav_roll  = msg.roll
    self.uav_pitch = msg.pitch
  #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  #----- Subscriber for VIDEO FRAMES  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  def sub_cb__image_raw(self, msg):
    ## Verify that we received the needed variables
    try:
      self.uav_position, self.uav_roll, self.uav_alt
    except:
      self.get_logger().warn("The position as not be published wet.")
      sleep(0.25)
      return
    
    ## Don't try to use camera if the drone is tilted
    if uav_is_tilted(self.uav_roll, self.uav_pitch):
      self.get_logger().warn("The UAV is tilted, not taking into account targets positions.")
      return
    
    ## Get current frame
    at_time_uav_position = self.uav_position
    at_time_uav_alt = self.uav_alt
    frame = self.cv_bridge.imgmsg_to_cv2(msg)
    if frame is None:
      self.get_logger().warn(" > No images received from camera.")
      return
    
    ## Determine target centers
    aruco_centers = self.find_aruco_centers(frame)
    
    ## Comput offsets
    if aruco_centers is not None:
      for aruco_center in aruco_centers:
        id = aruco_center[0]
        center = aruco_center[1]
        
        ## Check if id is reconized
        if id not in [5,222]:
          self.get_logger().warn("This ARUCO tag is not used")
          continue
        
        ## Compute offset
        offset = self.offset_to_meters(1  , center-self.img_center)
        dist_to_target = np.linalg.norm(offset)
        
        ## Compute the target GPS position
        target_lat, target_lon = self.offset_to_target_position(self.uav_position, offset)
        msg_pub     = GpsPosition()
        msg_pub.lat = float(target_lat)
        msg_pub.lon = float(target_lon)
        msg_pub.alt = float(0.)
          
        if id==5:
          self.get_logger().info("PARK target detected.")
          self.pub__cam_park_pos.publish(msg_pub)
        elif id==222:
          self.get_logger().info("FIRE target detected.")
          self.pub__cam_fire_pos.publish(msg_pub)
          
        ## For debuging
        if debug_cams:
          self.get_logger().info(f"Offset (m): ({offset[0]:.3f}, {offset[1]:.3f}) => delta (m): {dist_to_target:.2f}")
          # self.get_logger().info(f"GPS coords uav->target: {at_time_uav_position}-({target_lat}, {target_lon}) => delta ({target_lat-at_time_uav_position[0]}, {target_lon-at_time_uav_position[1]})")
          ## Use this site to compare results : https://www.omnicalculator.com/other/latitude-longitude-distance
          cv2.line(frame, (int(self.img_center[0]), int(self.img_center[1])), (int(center[0]), int(center[1])), (0, 255, 0), 1)
    
    ## To publish images debugs
    if True:
      self.pub__img_debug.publish(self.cv_bridge.cv2_to_imgmsg(frame))
  
  ############################################################################################################################################################################################################################
  ##### TOOLS ############################################################################################################################################################################################################################
  #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  #----- Converts an offset in pixels in offset in meters using camera specifications and drone altitude ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  def offset_to_meters(self, altitude, offset):
    offset_m = np.array(offset)*altitude*self.constant_pixel_to_meters
    # offset_m = (19.06,10.72)
    return offset_m
  #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  #----- Compute the target position from offset in meters (for short distances) ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  def offset_to_target_position(self, uav_position, offset):
    # ref: https://stackoverflow.com/questions/7477003/calculating-new-longitude-latitude-from-old-n-meters (it as errors, read the comments)
    # new_latitude  = latitude  + (dy / r_earth) * (180 / pi);
    # new_longitude = longitude + (dx / r_earth) * (180 / pi) / cos(NEW_latitude * pi/180);
    target_lat = uav_position[0] + offset[0] * CONV
    target_lon = uav_position[1] + offset[1] * CONV / np.cos(target_lat*TO_RAD)
    return (target_lat, target_lon)
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
    # executor = rclpy.executors.SingleThreadedExecutor()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=100)
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