#!/usr/bin/env python3
 
# Import general params
from time import sleep
from datetime import datetime
import time
import os
import cv2
from popeye.PARAMS_utils import *
from flask import Flask, Response
 
#########################################################################################################################################################################################################
##### Start video stream with webcam and display port as argument #####################################################################################################################################################################
# app = Flask(__name__)
# @app.route('/video_feed')
# def cam_start_stream(self,webcam,port):
#     # list_cameras()
   
#     if __name__ == '__main__':
#         app.run(host='0.0.0.0', port=5001)
   
# def cam_list_cameras(self,webcam,max_index=5,):
#     print("Recherche de caméras disponibles...")
#     for i in range(max_index):
#         if webcam is not None and webcam.isOpened():
#             print(f"Caméra trouvée à l'index {i}")
#             webcam.release()
#         else:
#             print(f" Pas de caméra à l'index {i}")
 
# def cam_generate_frames(self,webcam):
#     while True:
#         success, frame = webcam.read()
#         if not success:
#             break
#         else:
#             ret, buffer = cv2.imencode('.jpg', frame)
#             frame = buffer.tobytes()
#             yield (b'--frame\r\n'
#                 b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
 
# def cam_video_feed(self):
#     return Response(cam_generate_frames(),
#                     mimetype='multipart/x-mixed-replace; boundary=frame')
# def cam_stop_stream(self):
#         with self.lock:
#             self.streaming = False
#             if self.webcam:
#                 self.webcam.release()
#         return "Stream stopped", 200
 

#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#----- Function to LIST CAMERA ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def list_cameras(max_index=5):
        print("Looks for USB cameras...")
        for i in range(max_index):
            cap = cv2.VideoCapture(i)
            if cap is not None and cap.isOpened():
                print(f"Caméra trouvée à l'index {i}")
                cap.release()
            else:
                print(f" Pas de caméra à l'index {i}")
 
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#----- Function to TAKE PHOTO ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def cam_take_screenshot(webcam):
    _, imageFrame = webcam.read()
    now       = datetime.now()
    timestamp = now.strftime("%Y%m%d_%H%M%S") + f"{int(now.microsecond / 1000):03d}"
   
    # directory  = "/home/linux/ros2_DUAV/log_popeye/screenshots/"
    directory  = "/home/step/ros2_DUAV/log_popeye/screenshots/"
    # directory  = "/home/duav/ros2_DUAV/log_popeye/screenshots/"
    ## Checking if directory exists
    if not os.path.exists(directory):
        raise FileNotFoundError(f"Le dossier de destination n'existe pas : {directory}")
    filename   = directory+f"{timestamp}.png"
    cv2.imwrite(filename, imageFrame)
    return (filename)

#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#----- Function to TAKE VIDEO ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def cam_take_videowebcapture(webcam, image_shape, record_seconds):
    ## Video parameter
    ret, _ = webcam.read()
    height, width, _ = image_shape
    fps = 30
    
    ## Specify directory
    # directory  = "/home/linux/ros2_DUAV/log_popeye/videocaptures/"
    directory = "/home/step/ros2_DUAV/log_popeye/videocaptures/"
    # directory  = "/home/duav/ros2_DUAV/log_popeye/videocaptures/"
    if not os.path.exists(directory):
        print(f"{YELLOW}VIDEO TAKE save folder '{directory}' does not exits{RESET}")
        return False
    now = datetime.now()
    timestamp = now.strftime("%Y%m%d_%H%M%S") + f"{int(now.microsecond / 1000):03d}"
    filename = directory+f"{timestamp}.avi"
    
    ## Encode video
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    out = cv2.VideoWriter(filename, fourcc, fps, (width, height))
    
    ## Record
    print("Video recording started")
    start_time = time.time()
    while True:
        ret, frame = webcam.read()
        if not ret:
            break
        out.write(frame)
        if time.time() - start_time >= record_seconds:
            break
        
    out.release()
    return True