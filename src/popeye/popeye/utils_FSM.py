#!/usr/bin/env python3
### To get an understanding of the FSM, please refer to the documentation:
# DOC: https://python-statemachine.readthedocs.io/en/latest/actions.html

# Import general params
from time import sleep
from popeye.utils_PARAMS import *
import multiprocessing
import threading
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
    
    ### Workshop 1
    ws2__wait_for_GPS_coor    = State('Workshop2__WaitForGPSCoor')
    ws2__ready                = State('Workshop2__Ready')
    ws2__takeoffed            = State('Workshop2__Takeoffed')
    ws2__repositioned         = State('Workshop2__Repositioned')
    ws2__fire_hydrant_dropped = State('Workshop2__FireHydrantDropped')
    ws2__rtl                  = State('Workshop2__Rtl')
    
    ### Workshop 1
    ws1__ready         = State('Workshop1__Ready')
    ws1__takeoffed     = State('Workshop1__Takeoffed')
    ws1__repositioned  = State('Workshop1__Repositioned')
    ws1__landed        = State('Workshop1__Landed')

    ###### EVENTS ################################################################################################################################################################################################################################################################################################################################################
    ### Start and end events
    event = Event(idle.to(terminated,              cond=lambda: PopeyeFSM.option=="0")
                 | idle.to(ws2__wait_for_GPS_coor, cond=lambda: PopeyeFSM.option=="1")
                 | idle.to(ws1__ready,             cond=lambda: PopeyeFSM.option=="2"))
    ### Workshops
    event_WS2 = Event(ws2__wait_for_GPS_coor.to(ws2__ready)
                     | ws2__ready.to(ws2__takeoffed)
                     | ws2__takeoffed.to(ws2__repositioned)
                     | ws2__repositioned.to(ws2__fire_hydrant_dropped)
                     | ws2__fire_hydrant_dropped.to(ws2__rtl) 
                     | ws2__rtl.to(idle))  
    event_WS1 = Event(ws1__ready.to(ws1__takeoffed)
                     | ws1__takeoffed.to(ws1__repositioned)
                     | ws1__repositioned.to(ws1__landed)
                     | ws1__landed.to(idle))
    
    def __init__(self, node):
        self.node = node
        super(PopeyeFSM, self).__init__()
        
    ###### MENUS ####################################################################################################################################################################################################################################################################################################################################################################################################################################
    def menu_idle(self):
        while True:
            print("\n[FSM] ----------------- POPEYE MENU -----------------")
            print("[FSM] 1- Workshop FireFighter")
            print("[FSM] 2- Workshop 1")
            print("[FSM] 0- Terminate POPEYE")
            choice = input("\n[FSM] Select an option: ")
            print("[FSM] -----------------------------------------------")
            if choice in "012":
                PopeyeFSM.option = choice
                break
            print(f"{YELLOW}[FSM] Invalid option. Please select a valid number.{RESET}")
    def menu_action(self):
        while True:
            print("\n[FSM] ----------------- CONTROL MENU -----------------")
            print("[FSM] 0- Cancel Action")
            choice = input("\n[FSM] Select an option: ")
            print("[FSM] -----------------------------------------------")
            if choice in "0":
                break
            print(f"{YELLOW}[FSM] Invalid option. Please select a valid number.{RESET}")
            
        
    ###### STATE AND TRANSITION ACTIONS ####################################################################################################################################################################################################################################################################################################################################################################################################################################
    ### Start and end states
    @idle.enter
    def on_enter__idle(self):
        self.menu_idle()
        self.send('event')
    @terminated.enter
    def on_enter__terminated(self):
        print("\n[FSM] As terminated. Goodbye!")
        
    ### Firefighter states
    @ws2__wait_for_GPS_coor.enter
    def ws2_on_enter__wait_for_GPS_coor(self):
        print("[FSM] > WAIT FOR TARGET COORDONATES.")
        self.ws2__lon = DEFAULT_LON
        self.ws2__lat = DEFAULT_LAT
        self.ws2__alt = DEFAULT_ALT
        print("[FSM] > TARGET COORDONATES ACQUIRED.")
        self.send("event_WS2")
    @ws2__ready.enter
    def ws2_on_enter__ready(self):
        print("\n[FSM] > GETTING READY.")
        self.node.call__set_mode("GUIDED")
        self.node.call__arm(False)
        print("[FSM] > READY.")
        self.send("event_WS2")
    @ws2__takeoffed.enter
    def ws2_on_enter__takeoff(self):
        print("\n[FSM] > TAKING OFF.")
        action = threading.Thread(target=self.node.call__takeoff, args=[5.])
        menu   = threading.Thread(target=self.menu_action)
        action.start()
        menu.start()
        while action.is_alive():
            if not menu.is_alive():
                self.node.cancel_action = True
                break
            sleep(0.25)
        print("[FSM] > TAKEOFFED.")
        self.send("event_WS2")
    @ws2__repositioned.enter
    def ws2_on_enter__repositioned(self):
        print("\n[FSM] > REPOSITIONING.")
        # self.node.call__reposition(self.ws2__lat, self.ws2__lon, self.ws2__alt)
        action = threading.Thread(target=self.node.call__reposition, args=[self.ws2__lat, self.ws2__lon, self.ws2__alt])
        menu   = threading.Thread(target=self.menu_action)
        action.start()
        menu.start()
        while action.is_alive():
            if not menu.is_alive():
                self.node.cancel_action = True
                break
            sleep(0.25)
        print("[FSM] > REPOSITIONED.")
        self.send("event_WS2")
    @ws2__fire_hydrant_dropped.enter
    def ws2_on_enter__dropped(self):
        print("\n[FSM] > DROPPING.")
        # self.node.call__drop(self)
        print("[FSM] > DROPPED.")
        self.send("event_WS2")
    @ws2__rtl.enter
    def ws2_on_enter__rtl(self):
        print("\n[FSM] > Doing RTL.")
        self.node.call__rtl()
        print("[FSM] > RTL done.")
        self.send("event_WS2")
        
    ### Workshop 1 states
    @ws1__ready.enter
    def ws1_on_enter__ready(self):
        print("\n[FSM] > GETTING READY.")
        print("[FSM] ----------------- CHOSE REPOSITION COORDONATES -----------------")
        self.ws1__lon = float(input("[FSM] Lon: "))
        self.ws1__lat = float(input("[FSM] Lat: "))
        self.ws1__alt = float(input("[FSM] Alt: "))
        print("[FSM] ----------------------------------------------------------------")
        self.node.call__set_mode("GUIDED")
        self.node.call__arm(False)
        print("[FSM] > READY.")
        self.send("event_WS1")
    @ws1__takeoffed.enter
    def ws1_on_enter__takeoff(self):
        print("\n[FSM] > TAKING OFF.")
        self.node.call__takeoff(10)
        self.send("event_WS1")
        print("[FSM] > TAKEOFFED.")
    @ws1__repositioned.enter
    def ws1_on_enter__repositioned(self):
        print("\n[FSM] > REPOSITIONING.")
        self.node.call__reposition(self.ws1__lat, self.ws1__lon, self.ws1__alt)
        print("[FSM] > REPOSITIONED.")
        self.send("event_WS1")
    @ws1__landed.enter
    def ws1_on_enter__landed(self):
        print("\n[FSM] > LANDING.")
        self.node.call__land()
        print("[FSM] > LANDED.")
        self.send("event_WS1")