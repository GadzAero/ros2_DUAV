#!/usr/bin/env python3

# General importation
from popeye.PARAMS_utils import *
# Import Intefaces
from interfaces.msg import GpsPosition, TaskParams, Task
from interfaces.action import ExecuteTask

############################################################################################################################################################################################################################
#####  Python menu configs ############################################################################################################################################################################################################################
tasks = {
    "reposition": {
        "description": "Move to a user define GPS position",
        "params": [ {"name": "target_position", "type": "GpsPosition"} ]
    },
    "idle": {
        "description": "The drone has to stay idle",
        "params": [ {"name": "target_position", "type": "GpsPosition"} ]
    },
    "WS1": {
        "description": "GoTo and land",
        "params": [ {"name": "target_position", "type": "GpsPosition"} ]
    },    
    "WS2": {
        "description": "Workshop FireFighter",
        "params": [ ]
    },        
    "WS3": {
        "description": "Precision landing",
        "params": [  ]
    },        
    "Payload action 1": {
        "description": "Open (drop) payload",
        "params": [  ]
    },  
    "Payload action 2": {
        "description": "Close (reload) payload",
        "params": [  ]
    },  
    "Test 1": {
        "description": "Change Mode to ready and arm",
        "params": [  ]
    },  
    "Test 2": {
        "description": "Ready -> Takeoff (6m) -> Land",
        "params": [  ]
    },  
    "Test 3": {
        "description": "Ready -> Takeoff (6m) -> Search_square -> RTL",
        "params": [  ]
    },  
    "Test 4": {
        "description": "Ready -> Takeoff (6m) -> Asserv_cam_park -> RTL",
        "params": [  ]
    },  

}

############################################################################################################################################################################################################################
##### Functions to uses on the python config ############################################################################################################################################################################################################################
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#----- Function to CREATE THE USER MENU for tasks ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def task_menu():
    print("\n=== POPEYE Tasks Menu ===")
    for i, key in enumerate(tasks.keys()):
        print(f"{i+1}. {key} - {tasks[key]['description']}")
    print("0. Exit\n")
    choice = input("Choice: ")
    if choice == "0":
        print("Exiting.")
    elif choice in "1234567891011":
        return int(choice)
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#----- Function to CHOSE PARAMS for tasks ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def create_task_msg(choice):
    try:
        ## Get all task keys and names
        task_name = list(tasks.keys())[choice-1]
        task_data = tasks[task_name]
        ## Get args needed for the tasks
        args = []
        for param in task_data["params"]:
            pval = ask_for_input(param)
            task_arg_msg = TaskParams()
            if param["type"] == "GpsPosition":
                task_arg_msg.gps_position = pval
            args.append(task_arg_msg)
        ## Return the task
        return Task(task_name=task_name, arguments=args)
    except Exception as e:
        print(f"{YELLOW} Could not create the task : '{e}'{RESET}")
        return None
#---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#----- Function to ASK FOR INPUT from the user ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
def ask_for_input(param):
    ptype = param["type"]
    name  = param["name"]
    if ptype == "GpsPosition":
        x = float(input(f"  {name}.lat: "))
        y = float(input(f"  {name}.lon: "))
        z = float(input(f"  {name}.alt: "))
        return GpsPosition(lat=x, lon=y, alt=z)
    else:
        raise ValueError(f"Unsupported type: {ptype}")