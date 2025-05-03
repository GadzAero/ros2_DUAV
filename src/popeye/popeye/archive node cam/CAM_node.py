#!/usr/bin/env python3 

## Import standard utils
from time import sleep
## Import ROS2 utils
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
## Import Intefaces
from interfaces.srv import TakePhoto, TakeVideo
from interfaces.msg import Fire, GpsPosition, Attitude, Deltatarget, Targetpos
## Import ML utils
import matplotlib.pyplot as plt
import pathlib
import cv2
import numpy as np
## import torch
from PIL import Image
import warnings
import math
import time
## Import Hough transform utils
import numpy as np 
import math 
import cv2 
## Import CAM utils
import popeye.CAM_utils as cam_utils

## DEBUG
DEBUG = True

#####################################################################################################################################################################
##### Node for camera use #####################################################################################################################################################################
class CAMNode(Node):
    def __init__(self):
        super().__init__('CAM_node', namespace='POPEYE')
       
        ### SUBSCRIBERS
        self.sub__position = self.create_subscription(GpsPosition, 'position', self.sub_cb__position, 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.sub__attitude = self.create_subscription(Attitude, 'attitude', self.sub_cb__attitude, 10, callback_group=MutuallyExclusiveCallbackGroup())
       
        ### SERVICES
        self.srv__take_photo = self.create_service(TakePhoto, 'take_photo', self.srv_cb__take_photo, callback_group=MutuallyExclusiveCallbackGroup())
        self.srv__take_video = self.create_service(TakeVideo, 'take_video', self.srv_cb__take_video, callback_group=MutuallyExclusiveCallbackGroup())

        ### PUBLISHERS
        self.pub__target_pos   = self.create_publisher(Targetpos,   'target_pos',       10, callback_group=MutuallyExclusiveCallbackGroup())
        self.pub__delta_target = self.create_publisher(Deltatarget, 'distance_heading', 10, callback_group=MutuallyExclusiveCallbackGroup())
       
        ### TIMERS
        self.timer__white_search = self.create_timer(1, self.timer_cb__white_search, callback_group=MutuallyExclusiveCallbackGroup())

        #### WEBCAM INIT 
        ## For test
        # self.webcam = cv2.VideoCapture("/home/linux/ros2_DUAV/src/popeye/popeye/videos/videotest.mp4")
        self.webcam = cv2.VideoCapture("/home/step/ros2_DUAV/src/popeye/popeye/videos/videotest.mp4")
        ## Real
        # self.webcam = cv2.VideoCapture(0)
        _, self.image = self.webcam.read()
        while self.image is None:
            self.get_logger().warn(f"Camera can't be readed. Waiting again...")
            sleep(1)
        ## Compute camera parameters
        self.img_height, self.img_width, self.channels = self.image.shape
        self.image_shape = (self.img_height, self.img_width, self.channels)
        FOV_rad = math.radians(63.7)
        self.constant_pixel_to_meters = 2*math.tan(FOV_rad/2) / self.img_width  
        
        self.get_logger().info(" > NODE CAM__node STARTED.")
        
    ############################################################################################################################################################################################################################
    ##### SERVICES CALLBACKS ############################################################################################################################################################################################################################
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Service server to TAKE PHOTO ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def srv_cb__take_photo(self, request, response):
        print()
        self.get_logger().info(f"> Call service TAKE_PHOTO")
        if cam_utils.cam_take_screenshot(self.webcam):
            response.success = True
            self.get_logger().info("      -> Success.")
        else:
            response.success = False
            self.get_logger().warning("      -> Failure")
        return response
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Service server to TAKE VIDEO ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def srv_cb__take_video(self, request, response):
        print()
        self.get_logger().info(f"> Call service TAKE_VIDEO (seconds:{request.seconds})")
        if cam_utils.cam_take_videowebcapture(self.webcam, self.image_shape, request.seconds):
            response.success = True
            self.get_logger().info("      -> Success.")
        else:
            response.success = False
            self.get_logger().warning("      -> Failure")
        return response

    ############################################################################################################################################################################################################################
    ##### TIMERS CALLBACKS ############################################################################################################################################################################################################################
    def timer_cb__white_search(self):
        ## Colors to detect
        self.filtre_inf_blanc = np.array([0, 0, 200])     # faible saturation, forte valeur
        self.filtre_sup_blanc = np.array([180, 50, 255])
        
        #### While camera is active
        while True:  
            ## Lecture image cam
            _, frame = self.webcam.read() 
            if frame is None:
                self.get_logger().warn(f"Camera can't be readed.")
                return
        
            ## Set range for white color and define mask 
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
            white_lower = np.array(self.filtre_inf_blanc, np.uint8) 
            white_upper = np.array(self.filtre_sup_blanc, np.uint8) 
            white_mask  = cv2.inRange(hsv_frame, white_lower, white_upper) 
            kernel      = np.ones((5, 5), "uint8") 
            
            ## Création de contours
            cv2.dilate(white_mask, kernel) 
            cv2.bitwise_and(frame, frame, mask = white_mask) 
            contours, _ = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            img_center = (self.img_width/2, self.img_height/2)
            img_center_x, img_center_y = map(int, img_center)
            cv2.rectangle(frame, (img_center_x-10, img_center_y-10), (img_center_x+10, img_center_y+10), (0, 255,0), 2) 
        
            ## For each detected white aera
            for contour in contours: 
                area = cv2.contourArea(contour) 
                if(area > 5000):
                    ## Compute withe aera bounding rectangle 
                    rect_x, rect_y, rect_w, rect_h = cv2.boundingRect(contour)
                    centre=(rect_x + rect_w/2,rect_y+rect_h/2)
                    
                    ## Compute offsets
                    offset_px = (centre[0]-img_center[0], img_center[1]-centre[1])
                    offset    = self.conversion_pixel_metre(self.uav_alt, offset_px)
                    
                    ## Plot debug
                    cv2.line(frame,(int(img_center[0]), int(img_center[1])),(int(centre[0]), int(centre[1])), (255, 0, 255), 1)   
                    cv2.rectangle(frame, (rect_x, rect_y), (rect_x + rect_w, rect_y + rect_h), (0, 0, 255), 2)  
                    cv2.putText(frame, "Zone blanche", (rect_x, rect_y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255)) 
                   
                    ## Publish offset
                    self.tgt_distheanding = self.offset_xy_to_distandheading(offset   )
                    self.publication_offset()
                    ## Publish taget position
                    self.target_pos=self.offset_distheading_meters_to_GPS()
                    self.publication_target_position()
                #break

            if DEBUG:
                cv2.imshow("GadzAero Thermal detection software", frame) 
                if cv2.waitKey(1) & 0xFF == ord('q'): #Pressser q pour terminer
                    self.image.release() 
                    cv2.destroyAllWindows() 
                    break
            sleep(0.1)
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- PUBLICATION METHODS ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def publication_offset(self):
        msg_pub         = Deltatarget()
        msg_pub.dist    = self.target_dist
        msg_pub.heading = self.target_heading
        self.pub__delta_target.publish(msg_pub)
        # self.get_logger().info(f"TARGET > Target_dist: {self.target_dist} Target_heading: {self.target_heading} ")
    def publication_target_position(self):
        msg_pub          = Targetpos()
        msg_pub.lat_fire = self.target_pos[0]
        msg_pub.lon_fire = self.target_pos[1]
        self.pub__target_pos.publish(msg_pub)
        # self.get_logger().info(f"TGT_POS > Tgt_lat: {self.target_pos[0]} Tgt_lon: {self.target_pos[1]} ")
        
    ############################################################################################################################################################################################################################
    ##### SUBSCRIBERS CALLBACKS ############################################################################################################################################################################################################################
    def sub_cb__position(self, msg):
        self.uav_lat = msg.lat
        self.uav_lon = msg.lon
        self.uav_alt = msg.alt
    def sub_cb__attitude(self, msg):
        self.uav_yaw = msg.yaw

    ############################################################################################################################################################################################################################
    ##### CONVERSION FUNCTION ############################################################################################################################################################################################################################
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Converts an offset in pixels in offset in meters using camera specifications and drone altitude------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def conversion_pixel_metre(self,altitude,offset):
        ## Insert camera specifications for FOV 
        """OV2710 (format 1/2.7", dimensions 5856 µm × 3276 µm) avec différents objectifs possibles — par exemple un CS 5-50 mm pour le modèle ELP-USBFHD01M-SFV(5-50)."""
        ## OV2710 FOV at 5 mm focal (max unzoom)
        x_offset_pixel, y_offset_pixel = map(int,offset)
        x_offset_metres = x_offset_pixel*altitude*self.constant_pixel_to_meters   
        y_offset_metres = y_offset_pixel*altitude*self.constant_pixel_to_meters                              
        return (x_offset_metres, y_offset_metres)
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Converts an offset (whatever unit is given) in distance and heading using drone and camera orientation------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def offset_xy_to_distandheading(self,offset_xy):
        self.drone_heading = (math.degrees(self.uav_yaw)+360) % 360 
        self.offset_x,self.offset_y = offset_xy
        self.camera_heading = 0 #Degres
        ## Distance calculation
        self.target_dist = math.sqrt(self.offset_x**2+self.offset_y**2)
        ## Target heading calculation and correction for camera and drone orientation
        self.target_heading = math.degrees(math.atan2(self.offset_y, self.offset_x))
        ## Adding camera offset to the heading
        self.target_heading += self.camera_heading+self.drone_heading
        self.target_heading = (self.target_heading + 360) % 360
        return (self.target_dist, self.target_heading)
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Converts an offset in distance and heading to GPS coordinates using drone position as initial reference------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def offset_distheading_meters_to_GPS(self):
        self.drone_lat     = self.uav_lat
        self.drone_long    = self.uav_lon
        self.drone_heading = (math.degrees(self.uav_yaw)+360) % 360
        R = 6371000  
        # Conversion en radians
        self.drone_lat      = math.radians(self.drone_lat)
        self.drone_long     = math.radians(self.drone_long)
        self.target_heading = math.radians(self.target_heading)
        # Calcul
        self.target_lat = math.asin(math.sin(self.drone_lat) * math.cos(self.target_dist / R) + math.cos(self.drone_lat) * math.sin(self.target_dist / R) * math.cos(self.target_heading))
        self.target_lon = self.drone_long + math.atan2(math.sin(self.target_heading) * math.sin(self.target_dist / R) * math.cos(self.drone_lat), math.cos(self.target_dist / R) - math.sin(self.drone_lat) * math.sin(self.target_lat))

        # Conversion en degrés pour le résultat
        self.target_lat = math.degrees(self.target_lat)
        self.target_lon = math.degrees(self.target_lon)
        
        return (self.target_lat, self.target_lon)

    

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
