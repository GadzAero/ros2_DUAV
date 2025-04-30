#!/usr/bin/env python3

# Import general params
from time import sleep
from popeye.PARAMS_utils import *
import multiprocessing
import threading
# Import ROS2 utils
from rclpy.node import Node
import rclpy
# Import FSM utils
from statemachine import StateMachine, State, Event
from flask import Flask, Response
import cv2
from datetime import datetime
import time


###### EVENTS ################################################################################################################################################################################################################################################################################################################################################
### Individual tests

#########################################################################################################################################################################################################
##### Start video stream with webcam and display port as argument #####################################################################################################################################################################
app = Flask(__name__)
@app.route('/video_feed')
def cam_start_stream(self,webcam,port):
    # list_cameras()
    
    if __name__ == '__main__':
        app.run(host='0.0.0.0', port=5001)
    
def cam_list_cameras(self,webcam,max_index=5,):
    print("Recherche de caméras disponibles...")
    for i in range(max_index):
        if webcam is not None and webcam.isOpened():
            print(f"Caméra trouvée à l'index {i}")
            webcam.release()
        else:
            print(f" Pas de caméra à l'index {i}")

def cam_generate_frames(self,webcam):
    while True:
        success, frame = webcam.read()
        if not success:
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

def cam_video_feed(self):
    return Response(cam_generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')
def cam_stop_stream(self):
        with self.lock:
            self.streaming = False
            if self.webcam:
                self.webcam.release()
        return "Stream stopped", 200

#########################################################################################################################################################################################################
##### ACTION TO TAKE SCREENSHOTS #####################################################################################################################################################################
def cam_take_screenshot(self,webcam):
    _, imageFrame = self.webcam.read() 
    now       = datetime.now()
    timestamp = now.strftime("%Y%m%d_%H%M%S") + f"{int(now.microsecond / 1000):03d}"
    filename  = f"/home/linux/ros2_DUAV/log_popeye/screenshots/{timestamp}.png"
    print(filename)
    cv2.imwrite(filename, imageFrame)
    return (filename)

#########################################################################################################################################################################################################
##### ACTION TO START A PREDEFINED TIME VIDEOCAPTURE #####################################################################################################################################################################
def cam_take_videowebcapture(self,webcam,image_shape,record_seconds):
    ## Paramètres de la vidéo
    ret, image              = webcam.read()
    height, width, channels = image_shape
    fps                     =  30  # fallback si FPS = 0
    ## Naming the file with YYMMDD_HHMMSSms format
    now        = datetime.now()
    timestamp  = now.strftime("%Y%m%d_%H%M%S") + f"{int(now.microsecond / 1000):03d}"
    filename   = f"/home/linux/ros2_DUAV/log_popeye/videocaptures/{timestamp}.avi"
    ## Encoder
    fourcc     = cv2.VideoWriter_fourcc(*'MJPG')
    out        = cv2.VideoWriter(filename, fourcc, fps, (width, height))
    ##Timer_start and while entry
    start_time = time.time()
    while True:
        ret, frame = webcam.read()
        if not ret:
            break
        out.write(frame)
        if time.time() - start_time >= record_seconds:
            break
    webcam.release()
    out.release()
    cv2.destroyAllWindows()
    return (filename)









    
    ###### MENUS ####################################################################################################################################################################################################################################################################################################################################################################################################################################
