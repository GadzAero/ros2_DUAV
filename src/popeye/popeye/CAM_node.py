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
from interfaces.msg import Fire, UavPosition, UavAttitude

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
        
        self.ML_init()
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


    #####################################################################################################################################################################
    ##### ML Algorithm functions #####################################################################################################################################################################
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to initialize the ML algorithm  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def ML_init(self):
        ## Image data and characteristics read
        self.ret, self.image = self.video.read()
        self.cam_height, self.cam_width, channels = self.image.shape
        self.centre_image=(self.cam_width/2, self.cam_height/2)
        
        warnings.filterwarnings("ignore", category=FutureWarning)
        ## Import of the predictive model trained for the specific pattern
        model = torch.hub.load("ultralytics/yolov5", "custom", 
                                path="/home/linux/ros2_DUAV/src/popeye/popeye/best.pt", 
                                force_reload=True)
        # model = torch.hub.load("ultralytics/yolov5", "custom", 
        #                         path="/home/linux/ros2_DUAV/src/popeye/popeye/best.pt")

        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        ## Déplacer le modèle sur le GPU
        #model.to(device)  
        

        ## Forcing a confidence low limit for detection
        # model.conf=0.2

    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function generating a new CV2 window  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def afficher_imge(self,source,nom_fenetre):
        cv2.namedWindow(nom_fenetre, cv2.WINDOW_NORMAL)  
        cv2.resizeWindow(nom_fenetre, 600, 400)
        cv2.imshow(nom_fenetre, source)
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function submitting an image to the predictive model to get a detection  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def inference(self,image):
        results = self.model(image)  # Au lieu de model.predict(image, size=640, conf=0.25)
        #results.show() #Cette fonction ralentit enormement A NE PAS UTILISER
        
        # Vérifier les résultats de détection
        boxes=[]

        for det in results.xywh[0]:  # Results pour la première image
            xmin, ymin, xmax, ymax, confidence, class_idx = det.tolist()
            
            #print(f"Detection: {class_idx} (Confidence: {confidence:.2f})")
            #print(f"Bounding box: ({xmin}, {ymin}, {xmax}, {ymax})")
            
            boxes.append([xmin,ymin,xmax,ymax,class_idx])
        return boxes
    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to add the detection overlay to the image and display it  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------          
    def traitement(self,image):
        torch.cuda.empty_cache()
        ## On récupere les zones d'interet par inférence de notre modele de ML (il fait une prediction par image)
        boxes=self.inference(image)
        ## On récupere les dimensions
        img_height= image.shape[0]
        img_width = image.shape[1]
        
        ##On définit une nouvelle image sur laquelle on dessinera les boites
        image_avec_overlay=image
        for box in boxes:
            x_cent, y_cent,width,height, classe = box
            ## On donne les coordonées de chaque box
            xmin=int(x_cent-width/2)
            xmax=int(x_cent+width/2)
            ymin=int(y_cent-height/2)
            ymax=int(y_cent+height/2)

            ## Ajout du cadre a l'aide des coordonnées calculées
            image_avec_overlay = cv2.rectangle(image_avec_overlay, (xmin, ymin),  
                                        (xmax, ymax),  
                                        (0, 0, 255), 2) 
            ## Ajout legende en haut du cadre 
            cv2.putText(image_avec_overlay, "Flamme", (xmin, ymin), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
                            (0, 0, 255))  
            ## Calcul offset du centre du cadre par rapport au centre de l'image (en pixel)
            self.offset__xy=(self.centre_image[0]-x_cent,self.centre_image[1]-y_cent)
        if len(boxes)>0:
            self.target_detected=True
        else: self.target_detected=False
        return image_avec_overlay
    


    #---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    #----- Function to convert the offset in meters  ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    def conversion_pixel_metre(self,altitude,offset):
        #Renseigner les données FOV en degres de la camera 
        """OV2710 (format 1/2.7", dimensions 5856 µm × 3276 µm) avec différents objectifs possibles — par exemple un CS 5-50 mm pour le modèle ELP-USBFHD01M-SFV(5-50)."""
        #FOV A 5 mm de focale (dézoom max)
        FOV_horizontal=math.radians(63.7)
        FOV_vertical=math.radians(37.9)
        
        resolution_horizontale = self.cam_height
        resolution_verticale   = self.cam_width
        
        largeur_champ = 2*altitude*math.tan(FOV_horizontal/2)
        hauteur_champ = 2*altitude*math.tan(FOV_vertical/2)
        
        taille_pixel_horizontal=largeur_champ/resolution_horizontale
        taille_pixel_vertical=hauteur_champ/resolution_verticale

        x_offset_pixel,y_offset_pixel=map(int,offset)
        
        x_offset_metres=x_offset_pixel*taille_pixel_horizontal
        y_offset_metres=y_offset_pixel*taille_pixel_vertical
        #Renvoie le tuple d'offset du centre de la detection par rapport au centre de l'image en metres
        return ((x_offset_metres,y_offset_metres))
    
    def offset_xy_to_distandheading(self,drone_heading,offset_xy):
        self.drone_heading= drone_heading #ATTENTION DOIT ETRE EN DEG
        self.offset_x,self.offset_y=offset_xy
        self.camera_heading=0 
        ## Distance calculation
        self.target_dist=math.sqrt(self.offset_x**2+self.offset_y**2)
        ## Target heading calculation and correction for camera and drone orientation
        self.target_heading  = math.degrees(math.atan2(self.offset_y, self.offset_x))
        self.target_heading += self.camera_heading+self.drone_heading
        self.target_heading  = (self.target_heading + 360) % 360

        return ((self.target_dist,self.target_heading))

    def publication_offset(self):
        msg_pub = Deltatarget()
        msg_pub.dist_to_target    = self.target_dist
        msg_pub.heading_to_target = self.target_heading
        msg_pub.is_detected       = self.target_detected
        self.pub__offset.publish(msg_pub)
        # self.get_logger().info(f"FIRE > Fire_lat: {self.fire_pos_lat} Fire_lon: {self.fire_pos_lon} Is_fire: {self.is_fire}")
    
    def offset_meters_to_GPS(self,offset_distheading):
        self.drone_lat     = self.lat
        self.drone_long    = self.lon
        self.drone_heading = self.yaw
        self.target_dist,self.target_heading= self.offset_xy_to_distandheading(self.drone_heading,self.offset_xy)
        R = 6371000  
        # Conversion en radians
        self.drone_lat = math.radians(self.drone_lat)
        self.drone_long = math.radians(self.drone_long)
        self.target_heading= math.radians(self.target_heading)
        # Calcul
        self.target_lat = math.asin(math.sin(self.drone_lat) * math.cos(self.target_dist / R) +
                        math.cos(self.drone_lat) * math.sin(self.target_dist / R) * math.cos(self.target_heading))

        self.target_lon= self.drone_long + math.atan2(math.sin(self.target_heading) * math.sin(self.target_dist / R) * math.cos(self.drone_lat),
                                math.cos(self.target_dist / R) - math.sin(self.drone_lat) * math.sin(self.target_lat))

        # Conversion en degrés pour le résultat
        self.target_lat = math.degrees(self.target_lat)
        self.target_lon = math.degrees(self.target_lon)
        
        return (self.target_lat, self.target_lon)


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