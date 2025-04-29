#!/usr/bin/env python3

## Import standard utils
from time import sleep
from popeye.PARAMS_utils import *
## Import ROS2 utils
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
## Import Intefaces
from interfaces.srv import SetMode, Arm, Rtl, Disarm, Drop
from interfaces.action import Takeoff, Land, Reposition
from interfaces.msg import Fire, UavPosition, UavAttitude, DeltaTarget, TargetPos
## Import ML utils
import matplotlib.pyplot as plt
import pathlib
import cv2
import numpy as np
# import torch
from PIL import Image
import warnings
import math
import time
## Import Hough transform utils
import numpy as np 
import math 
import cv2 

#####################################################################################################################################################################
#####################################################################################################################################################################
##### Node for camera use #####################################################################################################################################################################
class CAMNode(Node):
    def __init__(self):
        super().__init__('CAM_node', namespace='POPEYE')
        
        ### SERVICE CLIENTS
        self.cli_srv__set_mode = self.create_client(SetMode, 'set_mode', callback_group=MutuallyExclusiveCallbackGroup())
        
        ### SUBSCRIBERS 
        self.sub__position = self.create_subscription(UavPosition, 'position', self.sub_cb__uav_position, 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.sub__attitude = self.create_subscription(UavAttitude, 'attitude', self.sub_cb__uav_attitude, 10, callback_group=MutuallyExclusiveCallbackGroup())
        
        ### TIMERS 
        self.timer__white_search = self.create_timer(1, self.timer_cb__white_search, callback_group=MutuallyExclusiveCallbackGroup())

        ### PUBLISHERS 
        self.pub__target_pos   = self.create_publisher(TargetPos,   'target_pos',       10, callback_group=MutuallyExclusiveCallbackGroup())
        self.pub__delta_target = self.create_publisher(DeltaTarget, 'distance_heading', 10, callback_group=MutuallyExclusiveCallbackGroup())

    ############################################################################################################################################################################################################################
    ##### TIMERS CALLBACKS ############################################################################################################################################################################################################################
    def timer_cb__white_search(self):
        ## Colors to detect (Low staturation => high value)
        filter_inf_white = np.array([0, 0, 200])
        filter_sup_white = np.array([180, 50, 255])

        ## Use a recording
        self.webcam = cv2.VideoCapture("/home/step/ros2_DUAV/src/popeye/popeye/videos/videotest.mp4")
        ## Use a webcam
        # self.webcam = cv2.VideoCapture(0)
        if not self.webcam.isOpened():
            self.get_logger().warn("Error: Could not open video source.")
            return

        ## Get image caracteristics
        _, image = self.webcam.read()
        self.img_height, self.img_width, _ = image.shape
        img_center = (self.img_width/2, self.img_height/2)
        
        ###### Main loop for the camera feed and calculations after initialisation 
        while True:
            ## Set image to RGB
            _, frame = self.webcam.read() 
            hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
        
            ## Set range for white color and define mask 
            white_lower = np.array(filter_inf_white, np.uint8) 
            white_upper = np.array(filter_sup_white, np.uint8) 
            white_mask  = cv2.inRange(hsvFrame, white_lower, white_upper) 
            kernel      = np.ones((5, 5), "uint8") 
            
            ## Get images whites
            white_mask = cv2.dilate(white_mask, kernel) 
            cv2.bitwise_and(frame, frame, mask=white_mask) 
            
            ## Find contours
            contours, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
            ## Affiche un rectangle de 10 par 10 au centre vert
            x_center, y_center = map(int, img_center)
            frame = cv2.rectangle(frame, (x_center-10, y_center-10), (x_center+10, y_center+10), (0,255,0), 2)
        
            ###### For each detected contour
            for contour in contours: 
                area = cv2.contourArea(contour) 
                if(area > 5000):
                    ## Compute best fitting non-oriented rectangle
                    rectangle_x, rectangle_y, rectangle_w, rectangle_h = cv2.boundingRect(contour) 
                    centre = (rectangle_x + rectangle_w/2,rectangle_y+rectangle_h/2)
                    
                    ## Compute distance and heading (in meters and GPS coords) 
                    offset_px = (centre[0]-img_center[0], img_center[1]-centre[1])
                    offset = self.pixels_to_meters(self.uav_alt, offset_px)
                    self.tgt_distheanding = self.offset_xy_to_distandheading(offset)
                    self.target_pos       = self.offset_distheading_meters_to_GPS()
                    
                    ## Adding red frame over dected target
                    cv2.rectangle(frame, (rectangle_x, rectangle_y), (rectangle_x+rectangle_w, rectangle_y+rectangle_h), (0,0,255), 2) 
                    cv2.putText(frame, "White_aera", (rectangle_x, rectangle_y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255)) 
                    cv2.line(frame, (int(img_center[0]),int(img_center[1])), (int(centre[0]),int(centre[1])), (255,0,255), 1)    

                    ## Publish results
                    self.publish_target_offset()
                    self.publish_target_position()

            ## Plot image
            cv2.imshow("GadzAero Thermal detection software", frame) 
            # if cv2.waitKey(0.1) & 0xFF == ord('q'): 
            #     image.release() 
            #     cv2.destroyAllWindows() 
            #     break
            
            sleep(0.1)
    
    ############################################################################################################################################################################################################################
    ##### SUBSCRIBERS CALLBACKS ############################################################################################################################################################################################################################
    def sub_cb__uav_position(self, msg):
        self.uav_lat = msg.lat
        self.uav_lon = msg.lon
        self.uav_alt = msg.alt
    def sub_cb__uav_attitude(self, msg):
        self.uav_yaw = msg.yaw

    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to convert an lenght in px to meters using camera specs and uav alt ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def pixels_to_meters(self, altitude, lenght):
        ## Insert camera specifications for FOV 
        """OV2710 (format 1/2.7", dimensions 5856 µm × 3276 µm) avec différents objectifs possibles — par exemple un CS 5-50 mm pour le modèle ELP-USBFHD01M-SFV(5-50)."""
        ## OV2710 FOV at 5 mm focal (max unzoom)
        
        ## Compute camera data
        resolution_width  = self.img_height
        resolution_height = self.img_width
        field_width  = 2*altitude*math.tan(math.radians(63.7)/2)
        field_height = 2*altitude*math.tan(math.radians(37.9)/2)
        pixel_width  = field_height/resolution_height
        pixel_height = field_width/resolution_width
        
        print("-----POURQUOI PAS LES MEMES DIMENSIONS ????--------")
        print(pixel_height)
        print(pixel_width)
        print("---------- + a optimiser, on veut pas réitérer plein de calculs inutiles a chaque itérations =) --------------------------------------")
       
        ## Compute and convert lenght to meters
        x_offset_px, y_offset_px = map(int, lenght)
        x_offset = x_offset_px*pixel_height
        y_offset = y_offset_px*pixel_width
        return (x_offset, y_offset)
    
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Converts an offset (whatever unit is given) in distance and heading using drone and camera orientation------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def offset_xy_to_distandheading(self, offset_xy):
        self.drone_heading          = (math.degrees(self.uav_yaw)+360) % 360 
        self.offset_x, self.offset_y = offset_xy
        self.camera_heading         = 0 #Degres
        ## Distance calculation
        self.target_dist            = math.sqrt(self.offset_x**2+self.offset_y**2)
        ## Target heading calculation and correction for camera and drone orientation
        self.target_heading         = math.degrees(math.atan2(self.offset_y, self.offset_x))
        ## Adding camera offset to the heading
        self.target_heading        += self.camera_heading+self.drone_heading
        self.target_heading         = (self.target_heading + 360) % 360
        return ((self.target_dist, self.target_heading))
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Converts an offset in distance and heading to GPS coordinates using drone position as initial reference------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def offset_distheading_meters_to_GPS(self):
        self.drone_heading = (math.degrees(self.uav_yaw)+360) % 360
        R = 6371000  
        # Conversion en radians
        self.uav_lat = math.radians(self.uav_lat)
        self.uav_lon = math.radians(self.uav_lon)
        self.target_heading = math.radians(self.target_heading)
        # Calcul
        self.target_lat = math.asin(math.sin(self.uav_lat) * math.cos(self.target_dist / R) + math.cos(self.uav_lat) * math.sin(self.target_dist / R) * math.cos(self.target_heading))

        self.target_lon= self.uav_lon + math.atan2(math.sin(self.target_heading) * math.sin(self.target_dist / R) * math.cos(self.uav_lat), math.cos(self.target_dist / R) - math.sin(self.uav_lat) * math.sin(self.target_lat))

        # Conversion en degrés pour le résultat
        self.target_lat = math.degrees(self.target_lat)
        self.target_lon = math.degrees(self.target_lon)
        
        return (self.target_lat, self.target_lon)
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- PUBLICATION FUNCTIONS------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def publish_target_offset(self):
        msg_pub = DeltaTarget()
        msg_pub.dist    = self.target_dist
        msg_pub.heading = self.target_heading
        self.pub__delta_target.publish(msg_pub)
        self.get_logger().info(f"TARGET > Target_dist: {self.target_dist} Target_heading: {self.target_heading} ")
    def publish_target_position(self):
        msg_pub = TargetPos()
        msg_pub.lat_fire = self.target_pos[0]
        msg_pub.lon_fire = self.target_pos[1]
        self.pub__target_pos.publish(msg_pub)
        self.get_logger().info(f"TGT_POS > Tgt_lat: {self.target_pos[0]} Tgt_lon: {self.target_pos[1]} ")
    

#########################################################################################################################################################################################################
##### Node entry point #####################################################################################################################################################################
def main(args=None):
    rclpy.init(args=args)
    
    ### Creating the mutlithread executor
    node     = CAMNode()
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