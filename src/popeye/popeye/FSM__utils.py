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

##################################################################################################################################################################################################################################################################################################################################################################
##### FINITE STATE MACHINE FOR POPEYE ORDER CONTROL ##############################################################################################################################################################################################################################################################################################################
class PopeyeFSM(StateMachine): 
    ###### PARAMS ################################################################################################################################################################################################################################################################################################################################################
    event  = "idle"
    
    ###### STATES ################################################################################################################################################################################################################################################################################################################################################
    ### Start and end states
    idle       = State('Idle', initial=True)
    terminated = State('Terminated', final=True)
    
    ### Standard states
    std__ready          = State('std__Ready')
    std__ready_force    = State('std__ReadyForce')
    std__takeoffed      = State('std__Takeoffed')
    std__repositioned   = State('std__Repositioned')
    std__payload_drop   = State('std__PayloadDrop')
    std__payload_reload = State('std__PayloadReload')
    # std__landed       = State('std__Landed')
    # std__rtl          = State('std__Rtl')
    
    ### Workshop 1
    ws1__select_coord = State('ws1__SelectCoords')
    ws1__landed       = State('WS1__Landed')
    
    ### Workshop 2
    ws2__wait_for_GPS_coord = State('WS2__WaitForGPSCoord')
    ws2__get_fire_pos       = State('WS2__GetFirePos')
    ws2__rtl                = State('WS2__Rtl')

    ###### EVENTS ################################################################################################################################################################################################################################################################################################################################################
    ### Workshops
    event_terminate = Event(idle.to(terminated))
    event_WS1 = Event(idle.to(ws1__select_coord)
                     | ws1__select_coord.to(std__ready)
                     | std__ready.to(std__takeoffed)
                     | std__takeoffed.to(std__repositioned)
                     | std__repositioned.to(ws1__landed)
                     | ws1__landed.to(idle))
    event_WS2 = Event(idle.to(ws2__wait_for_GPS_coord)
                     | ws2__wait_for_GPS_coord.to(std__ready)
                     | std__ready.to(std__takeoffed)
                     | std__takeoffed.to(std__repositioned)
                     | std__repositioned.to(ws2__get_fire_pos)
                     | ws2__get_fire_pos.to(std__payload_drop)
                     | std__payload_drop.to(ws2__rtl) 
                     | ws2__rtl.to(idle))
    event_payload_reload = Event(idle.to(std__payload_reload)
                                | std__payload_reload.to(idle))
    event_payload_drop = Event(idle.to(std__payload_drop)
                              | std__payload_drop.to(idle))
    event_test2 = Event(idle.to(std__ready_force)
                       | std__ready_force.to(idle))
    
    def __init__(self, node):
        self.node = node
        super(PopeyeFSM, self).__init__()
        
    ###### MENUS ####################################################################################################################################################################################################################################################################################################################################################################################################################################
    def menu_idle(self):
        while True:
            print("\n[FSM] ----------------- POPEYE MENU -----------------")
            print("[FSM]")
            print("[FSM] 1- WS1: Go to and Land")
            print("[FSM] 2- WS2: Workshop FireFighter")
            print("[FSM] 3- WS3: Precision landing")
            print("[FSM] 4- Menu: payload actions")
            print("[FSM] 5- Menu: tests")
            print("[FSM] 0- Terminate POPEYE")
            choice = input("\n[FSM] Select an option: ")
            print("[FSM]")
            print("[FSM] -----------------------------------------------\n")
            if choice == "0":
                PopeyeFSM.event = "event_terminate"
            elif choice == "4":
                PopeyeFSM.event = self.menu_payload_actions()
            elif choice == "5":
                PopeyeFSM.event = self.menu_tests()
            elif choice in "123":
                PopeyeFSM.event = "event_WS"+choice
            else:
                print(f"{YELLOW}[FSM] Invalid option. Please select a valid number.{RESET}")
                continue
            ### Continue if choosen, else do aciton
            if PopeyeFSM.event != "event_idle":
                break
    def menu_tests(self):
        while True:
            print("\n[FSM] ----------------- POPEYE MENU -----------------")
            print("[FSM] 1- TEST1: Payload drop")
            print(f"[FSM] 2- TEST2: Change Mode {YELLOW}(GUIDED){RESET} and arm {YELLOW}(FORCE){RESET}")
            print("[FSM] 0- Go Back")
            choice = input("\n[FSM] Select an option: ")
            print("[FSM] -----------------------------------------------\n")
            if choice == "1":
                return "event_payload_drop"
            elif choice in "2":
                return "event_test"+choice
            elif choice == "0":
                return "event_idle"
            else:
                print(f"{YELLOW}[FSM] Invalid option. Please select a valid number.{RESET}")
                continue
    def menu_payload_actions(self):
        while True:
            print("\n[FSM] ----------------- POPEYE MENU -----------------")
            print("[FSM] 1- Open (drop) payload.")
            print("[FSM] 2- Close (reload) payload")
            print("[FSM] 0- Go Back")
            choice = input("\n[FSM] Select an option: ")
            print("[FSM] -----------------------------------------------\n")
            if choice == "1":
                return "event_payload_drop"
            if choice == "2":
                return "event_payload_reload"
            elif choice == "0":
                return "event_idle"
            else:
                print(f"{YELLOW}[FSM] Invalid option. Please select a valid number.{RESET}")
                continue
    def menu_action(self):
        while True:
            print("\n[FSM] ----------------- CONTROL MENU -----------------")
            print("[FSM] 0- Cancel Action")
            choice = input("\n[FSM] Select an option: ")
            print("[FSM] -----------------------------------------------")
            if choice in "0":
                break
            print(f"{YELLOW}[FSM] Invalid option. Please select a valid number.{RESET}")
    def menu_select_repo_coord(self):
        print("[FSM] ----------------- CHOSE REPOSITION COORDONATES -----------------")
        self.repo_lat = float(input("[FSM] Lat: "))
        self.repo_lon = float(input("[FSM] Lon: "))
        self.repo_alt = float(input("[FSM] Alt: "))
        print("[FSM] ----------------------------------------------------------------\n")
            
        
    ###### STATE AND TRANSITION ACTIONS ####################################################################################################################################################################################################################################################################################################################################################################################################################################
    ### Start and end states
    @idle.enter
    def on_enter__idle(self):
        PopeyeFSM.event = "idle"
        self.menu_idle()
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
        print("\n[FSM] > GETTING READY {YELLOW}FORCE{RESET}.")
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
        action = threading.Thread(target=self.node.call__reposition, args=[self.repo_lat, self.repo_lon, self.repo_alt])
        menu   = threading.Thread(target=self.menu_action)
        action.start()
        menu.start()
        while action.is_alive():
            if not menu.is_alive():
                self.node.cancel_action = True
                break
            sleep(0.25)
        print("[FSM] > REPOSITIONED.")
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
        
    ### WS1
    @ws1__select_coord.enter
    def ws1_on_enter__select_coord(self):
        self.menu_select_repo_coord()
        self.send(PopeyeFSM.event)
    @ws1__landed.enter
    def ws1_on_enter__landed(self):
        print("\n[FSM] > LANDING.")
        self.node.call__land()
        print("[FSM] > LANDED.")
        self.send(PopeyeFSM.event)
        
    ### WS2: Firefighter states
    @ws2__wait_for_GPS_coord.enter
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
    @ws2__get_fire_pos.enter
    def ws2_on_enter__get_fire_pos(self):
        print("[FSM] > SEARCHING FOR FIRE.")
        sleep(10)
        # Service to get fire position from camera (Felix)
        print("[FSM] > FOUND FIRE.")
        self.send(PopeyeFSM.event)
    @ws2__rtl.enter
    def ws2_on_enter__rtl(self):
        print("\n[FSM] > Doing RTL.")
        self.node.call__rtl()
        print("[FSM] > RTL done.")
        self.send(PopeyeFSM.event)