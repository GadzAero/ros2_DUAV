#!/usr/bin/env python3
# import sys 
# print(sys.version)

# Import standard utils
from time import sleep
# Import ROS2 utils
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
# Import Intefaces
from interfaces.srv import SetMode, Arm, Rtl, Disarm, Drop
from interfaces.action import Takeoff, Land, Reposition
from interfaces.msg import Fire, UavPosition, UavAttitude, Deltatarget, Targetpos


# Import ML utils
import matplotlib.pyplot as plt
import pathlib
import cv2
import numpy as np
import torch
from PIL import Image
import warnings
import math
import time

#Import Hough transform utils
import numpy as np 
import math 
import cv2 

# Import CAM utils
import popeye.CAM_utils as cam_utils

#####################################################################################################################################################################
##### Node for camera use #####################################################################################################################################################################
class CAMNode(Node):
    def __init__(self):
        super().__init__('CAM_node', namespace='POPEYE')
        
        ### SERVICE CLIENTS
        self.cli_srv__set_mode = self.create_client(SetMode, 'set_mode', callback_group=MutuallyExclusiveCallbackGroup())
        
        ### SUBSCRIBERS 
        self.sub__position = self.create_subscription(UavPosition, 'position', self.sub_cb__position, 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.sub__attitude = self.create_subscription(UavAttitude, 'attitude', self.sub_cb__attitude, 10, callback_group=MutuallyExclusiveCallbackGroup())
        
        ### TIMERS 
        # self.timer__fire_search = self.create_timer(1, self.timer_cb__fire_search, callback_group=MutuallyExclusiveCallbackGroup())
        # self.timer__white_search = self.create_timer(1, self.timer_cb__white_search, callback_group=MutuallyExclusiveCallbackGroup())

        ### PUBLISHERS 
        self.pub__target_pos   = self.create_publisher(Targetpos, 'target_pos', 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.pub__delta_target = self.create_publisher(Deltatarget, 'distance_heading', 10, callback_group=MutuallyExclusiveCallbackGroup())

        ## WEBCAM INIT 
        #self.webcam = cv2.VideoCapture("/home/linux/ros2_DUAV/src/popeye/popeye/videos/videotest.mp4")
        ## Utilisation feed direct webcam
        self.webcam = cv2.VideoCapture(0)
        
        self.ret, self.image                   = self.webcam.read()
        self.height, self.width, self.channels = self.image.shape
        self.image_shape                       =(self.height, self.width, self.channels)
        

        ## TEST FONCTIONS
        # cam_utils.cam_take_screenshot(self,self.webcam)
        cam_utils.cam_take_videowebcapture(self,self.webcam,self.image_shape, 10)

    def timer_cb__white_search(self):
        #Couleurs a detecter
        self.filtre_inf_blanc = np.array([0, 0, 200])     # faible saturation, forte valeur
        self.filtre_sup_blanc = np.array([180, 50, 255])

        self.nb_pt=0
       

        #Récupération des caractéristiques de l'image
        self.centre_image                      = (self.width/2, self.height/2)

        ##Calculating conversion constant pixel/meters
        FOV_rad                       = math.radians(63.7)
        resolution_horizontale        = self.width
        self.constant_pixel_to_meters = (2*math.tan(FOV_rad/2)) / resolution_horizontale  
        self.loop()
        # self.destroy_timer(self.timer__fire_search)

    
############################################################################################################################################################################################################################
##### SUBSCRIBERS CALLBACKS ############################################################################################################################################################################################################################
    def sub_cb__position(self, msg):
        self.lat = msg.lat
        self.lon = msg.lon
        self.alt = msg.alt
    
    def sub_cb__attitude(self, msg):
        self.yaw = msg.yaw

    

###########################################################################################################################################################################################################################
###### WHITE ZONES DETECTION ############################################################################################################################################################################################################################
    
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- This function contains the main loop for the camera feed and calculations after initialisation ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def loop(self):
        while 1:  
            ## Lecture image cam
            _, self.imageFrame = self.webcam.read() 
        
            ## Conversion RGB
            hsvFrame = cv2.cvtColor(self.imageFrame, cv2.COLOR_BGR2HSV) 
        
            ## Set range for white color and define mask 
            white_lower = np.array(self.filtre_inf_blanc, np.uint8) 
            white_upper = np.array(self.filtre_sup_blanc, np.uint8) 
            white_mask  = cv2.inRange(hsvFrame, white_lower, white_upper) 
            kernel      = np.ones((5, 5), "uint8") 
            
            # For white color 
            white_mask = cv2.dilate(white_mask, kernel) 
            res_white  = cv2.bitwise_and(self.imageFrame, self.imageFrame, mask = white_mask) 
            
            ## Création de contours
            contours, hierarchy = cv2.findContours(white_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
            ## Affiche un rectangle de 10 par 10 au centre vert
            x_c,y_c         = map(int,self.centre_image)
            cv2.rectangle(self.imageFrame, (x_c-10,y_c-10), (x_c + 10, y_c + 10), (0, 255,0), 2) 
        
            ## Pour chaque detection de chaque frame
            for pic, contour in enumerate(contours): 
                area = cv2.contourArea(contour) 
                #Critère sur l'aire minimale de detection en pixels
                if(area > 5000):
                    
                    self.nb_pt+=1 #Compteur de detections
                    x, y, w, h = cv2.boundingRect(contour)  #Position en xy du bord en haut a gauche du cadre, width et height
                    cv2.rectangle(self.imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 2) 
                    #Calcul de la position du centre du cadre (en px)
                    centre=(x + w/2,y+h/2)
                    #Adding legend 
                    cv2.putText(self.imageFrame, "Zone blanche", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255)) 
                    ## Calcul offset du centre du cadre par rapport au centre de l'image
                    self.offset           = (centre[0]-self.centre_image[0], self.centre_image[1]-centre[1])
                    ## Adding red frame over dected target
                    self.imageFrame       = cv2.line(self.imageFrame,(int(self.centre_image[0]), int(self.centre_image[1])),(int(centre[0]), int(centre[1])), (255, 0, 255), 1)    
                    ## Calculating the offset in meters
                    self.offset_metre     = self.conversion_pixel_metre(self.alt,self.offset)
                    ## Converting the offset in meters in distance and heading
                    self.tgt_distheanding = self.offset_xy_to_distandheading(self.offset_metre)
                    self.publication_offset()
                    ## Calculating target position
                    self.target_pos=self.offset_distheading_meters_to_GPS()
                    self.publication_target_position()
                    #time.sleep(1)
                #break

            cv2.imshow("GadzAero Thermal detection software", self.imageFrame) 
            if cv2.waitKey(1) & 0xFF == ord('q'): #Pressser q pour terminer
                self.image.release() 
                cv2.destroyAllWindows() 
                break
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Converts an offset in pixels in offset in meters using camera specifications and drone altitude------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def conversion_pixel_metre(self,altitude,offset):

        ## Insert camera specifications for FOV 
        """OV2710 (format 1/2.7", dimensions 5856 µm × 3276 µm) avec différents objectifs possibles — par exemple un CS 5-50 mm pour le modèle ELP-USBFHD01M-SFV(5-50)."""
        ## OV2710 FOV at 5 mm focal (max unzoom)
        x_offset_pixel,y_offset_pixel = map(int,offset)
        x_offset_metres               = x_offset_pixel*altitude*self.constant_pixel_to_meters   
        y_offset_metres               = y_offset_pixel*altitude*self.constant_pixel_to_meters                              
        return ((x_offset_metres,y_offset_metres))

    
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Converts an offset (whatever unit is given) in distance and heading using drone and camera orientation------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def offset_xy_to_distandheading(self,offset_xy):
        self.drone_heading          = (math.degrees(self.yaw)+360) % 360 
        self.offset_x,self.offset_y = offset_xy
        self.camera_heading         = 0 #Degres
        ## Distance calculation
        self.target_dist            = math.sqrt(self.offset_x**2+self.offset_y**2)
        ## Target heading calculation and correction for camera and drone orientation
        self.target_heading         = math.degrees(math.atan2(self.offset_y, self.offset_x))
        ## Adding camera offset to the heading
        self.target_heading        += self.camera_heading+self.drone_heading
        self.target_heading         = (self.target_heading + 360) % 360
        return ((self.target_dist,self.target_heading))
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Converts an offset in distance and heading to GPS coordinates using drone position as initial reference------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def offset_distheading_meters_to_GPS(self):
        self.drone_lat     = self.lat
        self.drone_long    = self.lon
        self.drone_heading = (math.degrees(self.yaw)+360) % 360
        R                  = 6371000  
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
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- PUBLICATION FUNCTIONS------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def publication_offset(self):
        msg_pub         = Deltatarget()
        msg_pub.dist    = self.target_dist
        msg_pub.heading = self.target_heading
        self.pub__delta_target.publish(msg_pub)
        self.get_logger().info(f"TARGET > Target_dist: {self.target_dist} Target_heading: {self.target_heading} ")
    
    def publication_target_position(self):
        msg_pub          = Targetpos()
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
