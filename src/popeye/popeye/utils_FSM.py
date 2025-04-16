#!/usr/bin/env python3
### To get an understanding of the FSM, please refer to the documentation:
# DOC: https://python-statemachine.readthedocs.io/en/latest/actions.html

# Import general params
from time import sleep
from popeye.utils_PARAMS import *
# Import ROS2 utils
from rclpy.node import Node
import rclpy
# Import FSM utils
from statemachine import StateMachine, State, Event


##################################################################################################################################################################################################################################################################################################################################################################
##### FINITE STATE MACHINE FOR POPEYE ORDER CONTROL ##############################################################################################################################################################################################################################################################################################################
class PopeyeFSM(StateMachine): 
    ###### PARAMS ################################################################################################################################################################################################################################################################################################################################################
    option = 0
    
    ###### STATES ################################################################################################################################################################################################################################################################################################################################################
    ### Start and end states
    idle       = State('Idle', initial=True)
    terminated = State('Terminated', final=True)
    
    ### Standard states
    # standard__idle        = State('Standard__Idle')
    # standard__change_mode = State('Standard__DoSetMode')
    # standard__armed       = State('Standard__Armed')
    # standard__takeoff     = State('Standard__Takeoff')
    # standard__reposition  = State('Standard__Reposition')
    # standard__landed      = State('Standard__Land')
    # standard__disarmed    = State('Standard__Disarmed')
    
    # ### Olive Following
    # olive_following__idle = State('OliveFollowing__Idle')
    
    ### Firefighter states
    fire_fighter__wait_for_GPS_coor      = State('FireFighter__WaitForGPSCoor')
    fire_fighter__write_approach_mission = State('FireFighter__WriteMission')
    fire_fighter__wait_for_validation    = State('FireFighter__WaitForValidation')
    fire_fighter__do_appraoch_mission    = State('FireFighter__DoMission')
    fire_fighter__target_aquisition      = State('FireFighter__TargetAquisition')
    fire_fighter__fire_hydrant_drop      = State('FireFighter__FireHydrantDrop')
    fire_fighter__rtl                    = State('FireFighter__Rtl')
    
    ### Workshop 1
    ws1__ready         = State('Workshop1__Ready')
    ws1__takeoffed     = State('Workshop1__Takeoffed')
    ws1__repositioned  = State('Workshop1__Repositioned')
    ws1__landed        = State('Workshop1__Landed')

    ###### EVENTS ################################################################################################################################################################################################################################################################################################################################################
    ### Start and end events
    event = Event(idle.to(terminated, cond=lambda: PopeyeFSM.option==0)
                 | idle.to(fire_fighter__wait_for_GPS_coor, cond=lambda: PopeyeFSM.option==1)
                 | idle.to(ws1__ready, cond=lambda: PopeyeFSM.option==2))
    event_FIREFIGHTING = Event(fire_fighter__wait_for_GPS_coor.to(fire_fighter__write_approach_mission)
                              | fire_fighter__write_approach_mission.to(fire_fighter__wait_for_validation)
                              | fire_fighter__wait_for_validation.to(fire_fighter__do_appraoch_mission)
                              | fire_fighter__do_appraoch_mission.to(fire_fighter__target_aquisition)
                              | fire_fighter__target_aquisition.to(fire_fighter__fire_hydrant_drop)
                              | fire_fighter__fire_hydrant_drop.to(fire_fighter__rtl)
                              | fire_fighter__rtl.to(idle))
    event_WS1 = Event(ws1__ready.to(ws1__takeoffed)
                     | ws1__takeoffed.to(ws1__repositioned)
                     | ws1__repositioned.to(ws1__landed)
                     | ws1__landed.to(idle))
    
    def __init__(self, node):
        self.node = node
        super(PopeyeFSM, self).__init__()
        
    ###### STATE AND TRANSITION ACTIONS ####################################################################################################################################################################################################################################################################################################################################################################################################################################
    ### Start and end states
    @idle.enter
    def on_ener__idle(self):
        print("\n[FSM] ----------------- POPEYE MENU -----------------")
        print("[FSM] 1- Workshop FireFighter")
        print("[FSM] 2- Workshop 1")
        print("[FSM] 0- Terminate POPEYE")
        choice = input("\n[FSM] Select an option: ")
        if choice.isdigit() and int(choice) in [0, 2]:
            PopeyeFSM.option = int(choice)
        else:
            print(f"{YELLOW}[FSM] Invalid option. Please select a valid number.")
        print("[FSM] -----------------------------------------------")
        self.send('event')
    @terminated.enter
    def on_enter__terminated(self):
        print("\n[FSM] As terminated. Goodbye!")
        
    ### Firefighter states
    @fire_fighter__wait_for_GPS_coor.enter
    def on_enter__wait_for_GPS_coor(self):
        print("[FSM] > WAIT FOR TARGET COORDONATES.")
        print("[FSM] > TARGET COORDONATES ACQUIRED.")
        sleep(1.5)
        self.send("event_FIREFIGHTING")
    @fire_fighter__write_approach_mission.enter
    def on_enter__write_approach_mission(self):
        print("[FSM] > WRITING APPROACH MISSION.")
        print("[FSM] > APPROACH MISSION WRITTEN.")
        sleep(1.5)
        self.send("event_FIREFIGHTING")
    @fire_fighter__wait_for_validation.enter
    def on_enter__wait_for_validation(self):
        print("[FSM] > WAITING FOR VALIDATION.")
        sleep(1.5)
        self.send("event_FIREFIGHTING")
    @fire_fighter__do_appraoch_mission.enter
    def on_enter__do_appraoch_mission(self):
        print("[FSM] > DOING APPROACH MISSION.")
        print("[FSM] > APPROACH MISSION DONE.")
        sleep(1.5)
        self.send("event_FIREFIGHTING")
    @fire_fighter__target_aquisition.enter
    def on_enter__target_aquisition(self):
        print("[FSM] > ACQUIRING TARGET.")
        print("[FSM] > TARGET ACQUIRED.")
        sleep(1.5)
        self.send("event_FIREFIGHTING")
    @fire_fighter__fire_hydrant_drop.enter
    def on_enter__fire_hydrant_drop(self):
        print("[FSM] > DROPPING FIRE HYDRANT.")
        print("[FSM] > FIRE HYDRANT DROPPED.")
        sleep(1.5)
        self.send("event_FIREFIGHTING")
    @fire_fighter__rtl.enter
    def on_enter__rtl(self):
        print("[FSM] > RETURNING TO LAUNCH.")
        print("[FSM] > RETURNED TO LAUNCH.")
        sleep(1.5)
        self.send("event_FIREFIGHTING")
        
    ### Workshop 1 states
    @ws1__ready.enter
    def on_enter__ready(self):
        print("\n[FSM] > GETTING READY.")
        print("[FSM] ----------------- CHOSE REPOSITION COORDONATES -----------------")
        global ready_lon, ready_lat, ready_alt
        ready_lon = 48.61 #float(input("[FSM] Lon: "))
        ready_lat = 2.39  #float(input("[FSM] Lat: "))
        ready_alt = 10    #float(input("[FSM] Alt: "))
        print("[FSM] -----------------------------------------------")
        self.node.call__set_mode("GUIDED")
        self.node.call__arm(False)
        print("[FSM] > READY.")
        self.send("event_WS1")
    @ws1__takeoffed.enter
    def on_enter__takeoff(self):
        print("\n[FSM] > TAKING OFF.")
        self.node.call__takeoff(10)
        self.send("event_WS1")
        print("[FSM] > TAKEOFFED.")
    @ws1__repositioned.enter
    def on_enter__reposition(self):
        print("\n[FSM] > REPOSITIONING.")
        global ready_lon, ready_lat, ready_alt
        self.node.call__reposition(ready_lon, ready_lat, ready_alt)
        del ready_lon, ready_lat, ready_alt
        print("[FSM] > REPOSITIONED.")
        self.send("event_WS1")
    @ws1__landed.enter
    def on_enter__landed(self):
        print("\n[FSM] > LANDING.")
        self.node.call__land()
        print("[FSM] > LANDED.")
        self.send("event_WS1")