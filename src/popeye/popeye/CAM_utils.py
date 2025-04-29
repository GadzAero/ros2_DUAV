#!/usr/bin/env python3
### To get an understanding of the FSM, please refer to the documentation:
# DOC: https://python-statemachine.readthedocs.io/en/latest/actions.html

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

##################################################################################################################################################################################################################################################################################################################################################################
##### FINITE STATE MACHINE FOR POPEYE ORDER CONTROL ##############################################################################################################################################################################################################################################################################################################
class CAMUtils(Utils): 
    ###### PARAMS ################################################################################################################################################################################################################################################################################################################################################

    
    ###### STATES ################################################################################################################################################################################################################################################################################################################################################
    ### Start and end states


    ###### EVENTS ################################################################################################################################################################################################################################################################################################################################################
    ### Individual tests
    def start_stream(self,cam_number,port):
        app = Flask(__name__)
        app2= Flask(__name__)
        cap = cv2.VideoCapture(0)  #Premiere camera USB detectée
        list_cameras()
        @app.route('/video_feed')
        if __name__ == '__main__':
            app.run(host='0.0.0.0', port=5001)

    def list_cameras(max_index=5):
        print("Recherche de caméras disponibles...")
        for i in range(max_index):
            cap = cv2.VideoCapture(i)
            if cap is not None and cap.isOpened():
                print(f"Caméra trouvée à l'index {i}")
                cap.release()
            else:
                print(f" Pas de caméra à l'index {i}")

    def generate_frames():
        while True:
            success, frame = cap.read()
            if not success:
                break
            else:
                ret, buffer = cv2.imencode('.jpg', frame)
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
    def video_feed():
        return Response(generate_frames(),
                        mimetype='multipart/x-mixed-replace; boundary=frame')
    def stop_stream():
            with self.lock:
                self.streaming = False
                if self.cap:
                    self.cap.release()
            return "Stream stopped", 200
    

    def do_take_screenshot(self,webcam):
        _, self.imageFrame = self.webcam.read() 
        now = datetime.now()
        timestamp = now.strftime("%Y%m%d_%H%M%S") + f"{int(now.microsecond / 1000):03d}"
        filename = f"{timestamp}.png"
        cv2.imwrite(filename, frame)
        return (filename)
    
    def do_videowebcapture(self,webcam,record_seconds):
        # Paramètres de la vidéo
        width      = int(webcam.get(cv2.webcam_PROP_FRAME_WIDTH))
        height     = int(webcam.get(cv2.webcam_PROP_FRAME_HEIGHT))
        fps        = webcam.get(cv2.webcam_PROP_FPS) or 30  # fallback si FPS = 0
        now        = datetime.now()
        timestamp  = now.strftime("%Y%m%d_%H%M%S") + f"{int(now.microsecond / 1000):03d}"
        filename   = f"{timestamp}.avi"
        fourcc     = cv2.VideoWriter_fourcc(*'MJPG')
        out        = cv2.VideoWriter(filename, fourcc, fps, (width, height))
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
