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

# Import ML utils
import matplotlib.pyplot as plt
import cv2
import numpy as np
import torch
from PIL import Image
import warnings
import math
import time
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
        self.timer__fire_search = self.create_timer(1, self.timer_cb__fire_search, callback_group=MutuallyExclusiveCallbackGroup())

    ############################################################################################################################################################################################################################
    ##### TIMERS CALLBACKS ############################################################################################################################################################################################################################
    def timer_cb__fire_search(self):
        ##Choix entre une vidéo préenregistrée ou une webcam en livefeed
        self.video=cv2.VideoCapture("/home/linux/ros2_DUAV/src/popeye/popeye/videos/videotest.mp4")
        # self.video=cv2.VideoCapture(0)
        cv2.namedWindow("Runway recognition DUAV", cv2.WINDOW_NORMAL)  # Permet de redimensionner la fenêtre
        cv2.resizeWindow("Runway recognition DUAV", 1080, 700)  # Ajuste la taille de la fenêtre (largeur x hauteur)
        
        # self.ML_init()
        while (self.video.isOpened()):
            print("video oppened")
            self.ret,self.frame = self.video.read()
            # frame = self.traitement(frame) #On réécrit chaque frame avec le traitement
            cv2.imshow("Runway recognition DUAV",self.frame)
            time.sleep(0.01)
            if cv2.waitKey(1) & 0XFF == ord('x') : #Fermeture si X est pressé
                break
        self.video.release()
        cv2.destroyAllWindows()
        
        self.destroy_timer(self.timer__fire_search)
    
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