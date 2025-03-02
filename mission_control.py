import airsim  # type: ignore
import multiprocessing as mp
import random
import time
import heapq
import os
import numpy as np
import cv2
import requests
import json
from ollama import chat # type: ignore
from pydantic import BaseModel # type: ignore

# project imports
import local_config
import single_drone_controller as sdc
from model_files.pull_model import load_model


MODEL = "llava:7b" # model from Ollama
URL = "http://localhost:11434/api/chat" 




# Waypoint class to store the name, location, and priority of each waypoint
# using a priority queue to prioritize waypoints
# location is a list of x, y, z Unreal Engine coordinates
# priority is 1 for user-assigned waypoints, 2 for dropped waypoints that must be revisited, and 3 for regular waypoints
class Waypoint:
    def __init__(self, name, x, y, z, priority):
        self.name = name
        self.x = x
        self.y = y
        self.z = z
        self.priority = priority

    def __lt__(self, other):
        return self.priority > other.priority 
    









class VLMOutput(BaseModel):
    waypoints: list[int] # id of the waypoint
    drone_id: int
    image_result: str # can be "heat signature detected", "no heat signature detected", "target confirmed", "target not confirmed"
    target_location: tuple # location of the target

    

# message history, starts off with explaining what the model 
# will be doing in the search and rescue mission
message_history = [
    {
        'role': 'user',
        'content': (
            "You are a drone operator conducting a simulated search and rescue mission.\n"
            "You have 5 drones at your disposal to locate a missing person.\n"
            "Your primary task is to create and assign waypoints for the drones, analyze captured images, and determine the target's location."
        ),
    },
    {
        'role': 'assistant',
        'content': "Understood. I am ready to assist in the search and rescue mission.",
    },
    {
        'role': 'user',
        'content': (
            "The search process follows these steps:\n"
            "1. You will receive a list of **points of interest (POIs)** with their locations, along with the current locations of the drones.\n"
            "2. Using this information, generate a **waypoint list** for the drones(they all travel together to each waypoint) to efficiently investigate the POIs.\n"
            "3. The drones will travel to each waypoint and capture **infrared (IR) images** to detect potential heat signatures.\n"
            "4. If a heat signature is detected, the drones will take **regular images** to visually confirm the target.\n"
            "5. The drones will transmit both IR and regular images to you for analysis.\n"
            "6. Based on your analysis, determine whether the detected heat signature matches the missing person.\n"
            "7. If confirmed, instruct **all drones** to converge on the target’s location to capture additional images and verify the identification.\n\n"
            "Your objective is to accurately locate and confirm the missing person’s position using drone imagery."
        ),
    },
]



# global queue for images to process








def parentController(drone_count):
    """ Parent process to send commands and receive status updates from drones. """
    mp.set_start_method('spawn')  # Windows-specific start method
    manager = mp.Manager() # manager to share data between processes


    #print("Loading VLM...")
    #load_model(MODEL)
    
    
    current_target_dictionary = manager.dict()  # Dictionary to store the current target waypoint of each drone
    status_dictionary = manager.dict() # Dictionary to store status of each drone
    target_found = mp.Value('b', False) # global variable to stop the search loop when the target is found
    processes = []  # List to store process references for each drone



    # POI1 is a small shack in front of the spawn area
    waypoint_queue = []
    heapq.heappush(waypoint_queue, Waypoint("POI1", 1000, 5000, 500, 3))

    



    # Create and start processes
    for x in range(1):
        drone_name = str(x)
        current_target_dictionary[drone_name] = None # an instance of the waypoint class
        status_dictionary[drone_name] = "INITIALIZING"
        p = mp.Process(target=sdc.singleDroneController, args=(drone_name, current_target_dictionary, status_dictionary, target_found))
        p.start()
        processes.append(p)
        print(f"Drone {drone_name} is initializing")




   
    while not all(status == "WAITING" for status in status_dictionary.values()):       
        print("Waiting for all drones to take off...")
        # for drone_name in status_dictionary:
        #     print(f"Drone {drone_name} status: {status_dictionary[drone_name]}")
        time.sleep(1)



    try:
        while not target_found.value:
            

            # first assign waypoints to any waiting drones
            for drone_name in status_dictionary:
                if status_dictionary[drone_name] == "WAITING":
                    if len(waypoint_queue) > 0:
                        next_waypoint = heapq.heappop(waypoint_queue)
                        current_target_dictionary[drone_name] = next_waypoint
                        print(f"Assigning waypoint {next_waypoint.name} to Drone {drone_name}")
                    else:
                        print(f"No waypoints available for Drone {drone_name}")




            time.sleep(1)  # Give some time between cycles
    
    except KeyboardInterrupt:
        for p in processes:
            p.terminate()

    # Wait for all processes to finish
    for p in processes:
        p.join()







if __name__ == '__main__':

    # directory to store pictures
    imgDir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'images')

    # create directory if it does not exist
    if not os.path.exists(imgDir):
        os.makedirs(imgDir)

    drone_count = 5
    parentController(drone_count)
