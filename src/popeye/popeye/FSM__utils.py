#!/usr/bin/env python3
### To get an understanding of the FSM, please refer to the documentation:
# DOC: https://python-statemachine.readthedocs.io/en/latest/actions.html

# Import general params
from time import sleep
from popeye.PARAMS_utils import *
# Import FSM utils
from statemachine import StateMachine, State, Event
# Import geopy utils
import geopy.distance as geodst
# Import interfaces
from interfaces.msg import GpsPosition, Task, TaskParams

##################################################################################################################################################################################################################################################################################################################################################################
##### FINITE STATE MACHINE FOR POPEYE ORDER CONTROL ##############################################################################################################################################################################################################################################################################################################
class PopeyeFSM(StateMachine): 
    ###### PARAMS ################################################################################################################################################################################################################################################################################################################################################
    event = "event_idle"
    
    ###### STATES ################################################################################################################################################################################################################################################################################################################################################
    ### Start and end states
    idle       = State('Idle', initial=True)
    terminated = State('Terminated', final=True)
    
    ### Standard states
    std__ready           = State('std__Ready')
    std__ready_force     = State('std__ReadyForce')
    std__takeoffed       = State('std__Takeoffed')
    std__repositioned    = State('std__Repositioned')
    std__square_search   = State('std__SquareSearch')
    std__asserv_cam_park = State('std__AsservCamPark')
    std__payload_drop    = State('std__PayloadDrop')
    std__payload_reload  = State('std__PayloadReload')
    std__take_photo      = State('std__TakePhoto')
    std__take_video      = State('std__TakeVideo')
    std__landed          = State('std__Landed')
    std__rtl             = State('std__Rtl')
    
    ### Workshop 2
    ws2__wait_for_fire_coords = State('WS2__WaitForFireCoords')
    ws2__get_cam_fire_pos     = State('WS2__GetCamFirePos')

    ###### EVENTS ################################################################################################################################################################################################################################################################################################################################################
    ### Workshops events
    event_terminate = Event(idle.to(terminated))
    event_idle = Event(idle.to(idle))
    # Reposition
    event_WS1 = Event(idle.to(std__ready)
                     | std__ready.to(std__takeoffed)
                     | std__takeoffed.to(std__repositioned)
                     | std__repositioned.to(std__landed)
                     | std__landed.to(idle))
    # Firefight
    event_WS2 = Event(idle.to(ws2__wait_for_fire_coords)
                     | ws2__wait_for_fire_coords.to(std__ready)
                     | std__ready.to(std__takeoffed)
                     | std__takeoffed.to(std__repositioned)
                     | std__repositioned.to(ws2__get_cam_fire_pos)
                     | ws2__get_cam_fire_pos.to(std__square_search)
                     | std__square_search.to(std__payload_drop)
                     | std__payload_drop.to(std__rtl) 
                     | std__rtl.to(idle))
    # Skills
    event_payload_reload = Event(idle.to(std__payload_reload)
                                | std__payload_reload.to(idle))
    event_payload_drop = Event(idle.to(std__payload_drop)
                              | std__payload_drop.to(idle))
    ## Test events
    event_test2 = Event(idle.to(std__ready_force)
                       | std__ready_force.to(idle))
    event_test3 = Event(idle.to(std__take_photo)
                       | std__take_photo.to(idle))
    event_test4 = Event(idle.to(std__take_video)
                       | std__take_video.to(idle))
    event_test5 = Event(idle.to(std__ready)
                       | std__ready.to(std__takeoffed)
                       | std__takeoffed.to(std__landed)
                       | std__landed.to(idle))
    event_test6 = Event(idle.to(std__ready)
                       | std__ready.to(std__takeoffed)
                       | std__takeoffed.to(std__square_search)
                       | std__square_search.to(std__rtl)
                       | std__rtl.to(idle))
    event_test7 = Event(idle.to(std__ready)
                       | std__ready.to(std__takeoffed)
                       | std__takeoffed.to(std__asserv_cam_park)
                       | std__asserv_cam_park.to(std__rtl)
                       | std__rtl.to(idle))
    
    def __init__(self, node):
        self.node = node
        super(PopeyeFSM, self).__init__()
      
    ###### STATE AND TRANSITION ACTIONS ####################################################################################################################################################################################################################################################################################################################################################################################################################################
    ### Start and end states
    @idle.enter
    def on_enter__idle(self):
        PopeyeFSM.event = "event_idle"
        if self.node.task_name is None:
            print(f"{YELLOW}[FSM] No task topic was yet published{RESET}")
        elif self.node.task_name == "reposition":
            print("[FSM] Launching task 'reposition'")
            gps_repo = self.node.task_params[0].gps_position
            self.repo_lat = gps_repo.lat
            self.repo_lon = gps_repo.lon
            self.repo_alt = gps_repo.alt
            PopeyeFSM.event = "event_WS1"
        else:
            print(f"{YELLOW}[FSM] Task name does not exists{RESET}")
        sleep(1)
        self.send(PopeyeFSM.event)
    @terminated.enter
    def on_enter__terminated(self):
        print("\n[FSM] As terminated. Goodbye!")
        
    ### Standard states
    @std__ready.enter
    def std_on_enter__ready(self):
        print("\n[FSM] > GETTING READY.")
        self.node.call__set_mode("GUIDED")
        self.node.call__arm(False)
        print("[FSM] > READY.")
        self.send(PopeyeFSM.event)
        
    @std__ready_force.enter
    def std_on_enter__ready_force(self):
        print(f"\n[FSM] > GETTING READY {YELLOW}FORCE{RESET}.")
        self.node.call__set_mode("GUIDED")
        self.node.call__arm(True)
        print("[FSM] > READY.")
        self.send(PopeyeFSM.event)
        
    @std__takeoffed.enter
    def std_on_enter__takeoff(self):
        print("\n[FSM] > TAKING OFF.")
        self.node.call__takeoff(6)
        print("[FSM] > TAKEOFFED.")
        self.send(PopeyeFSM.event)
        
    @std__repositioned.enter
    def std_on_enter__repositioned(self):
        print("\n[FSM] > REPOSITIONING.")
        self.node.call__reposition(self.repo_lat, self.repo_lon, self.repo_alt)
        print("[FSM] > REPOSITIONED.")
        self.send(PopeyeFSM.event)
        
    @std__square_search.enter
    def std_on_enter__square_search(self):
        print("\n[FSM] > STARTING SQUARE SEARCH.")
        intersect = 0.4
        delta_lat = 0.0001714106980870156 *intersect
        delta_lon = 0.00023963062838783245 *intersect
        self.node.call__reposition(self.repo_lat+delta_lat, self.repo_lon+delta_lon, self.repo_alt)
        self.node.call__reposition(self.repo_lat+delta_lat, self.repo_lon-delta_lon, self.repo_alt)
        self.node.call__reposition(self.repo_lat-delta_lat, self.repo_lon-delta_lon, self.repo_alt)
        self.node.call__reposition(self.repo_lat-delta_lat, self.repo_lon+delta_lon, self.repo_alt)
        print("[FSM] > END SQUARE SEARCH.")
        self.send(PopeyeFSM.event)
        
    @std__asserv_cam_park.enter
    def std_on_enter__asserv_cam_park(self):
        print("\n[FSM] > START ASSERV CAM PARK.")
        print(f"\n[FSM] ---------------------------------------")
        if self.node.cam_park_pos is None:
            print(f"{YELLOW}Position not published yet.{RESET}")
        else:
            print(f"\n[FSM] > park_pos:{self.node.cam_park_pos}).")
            print(f"\n[FSM] > park_dist:{ geodst.distance(self.node.uav_pos, self.node.cam_park_pos).m }).")
            self.node.call__reposition(self.node.cam_park_pos, self.node.cam_park_pos, 6)
        sleep(2)
        print("[FSM] > END ASSERV CAM PARK.")
        self.send(PopeyeFSM.event)
        
    @std__payload_drop.enter
    def std_on_enter__payload_drop(self):
        print("\n[FSM] > DROPPING.")
        self.node.call__payload_drop()
        print("[FSM] > DROPPED.")
        self.send(PopeyeFSM.event)
        
    @std__payload_reload.enter
    def std_on_enter__payload_reload(self):
        print("\n[FSM] > PAYLOAD RELOADING (you have 5s).")
        self.node.call__payload_reload()
        print("[FSM] > PAYLOAD RELOADED.")
        self.send(PopeyeFSM.event)
        
    @std__take_photo.enter
    def std_on_enter__take_photo(self):
        print("\n[FSM] > TAKING PHOTO.")
        self.node.call__take_photo()
        print("[FSM] > PHOTO TAKEN.")
        self.send(PopeyeFSM.event)
        
    @std__take_video.enter
    def std_on_enter__take_video(self):
        print("\n[FSM] > TAKING VIDEO.")
        self.node.call__take_video(3)
        print("[FSM] > VIDEO TAKEN.")
        self.send(PopeyeFSM.event)
        
    @std__landed.enter
    def std_on_enter__landed(self):
        print("\n[FSM] > LANDING.")
        self.node.call__land()
        print("[FSM] > LANDED.")
        self.send(PopeyeFSM.event)
        
    @std__rtl.enter
    def std_on_enter__rtl(self):
        print("\n[FSM] > Doing RTL.")
        self.node.call__rtl()
        print("[FSM] > RTL done.")
        self.send(PopeyeFSM.event)
        
    ### WS2: Firefighter states
    @ws2__wait_for_fire_coords.enter
    def ws2_on_enter__wait_for_GPS_coor(self):
        print("[FSM] > WAIT FOR TARGET COORDONATES.")
        while not self.node.is_fire:
            continue
        self.repo_lat = self.node.lat_fire
        self.repo_lon = self.node.lon_fire
        self.repo_alt = 2.
        print(f"{YELLOW}[FSM] > FIRE SPOTTED > LAT:{self.repo_lat} LON:{self.repo_lon}.{RESET}")
        print("[FSM] > TARGET COORDONATES ACQUIRED.")
        self.send(PopeyeFSM.event)
    @ws2__get_cam_fire_pos.enter
    def ws2_on_enter__get_fire_pos(self):
        print("[FSM] > SEARCHING FOR FIRE.")
        sleep(10)
        # Service to get fire position from camera (Felix)
        print("[FSM] > FOUND FIRE.")
        self.send(PopeyeFSM.event)