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
import base64
import asyncio
import websockets
from queue import Queue
import threading

# project imports
import local_config
import single_drone_controller as sdc
from model_files.pull_model import load_model
from classes import Waypoint, Image, VLMOutput
from helper_functions import reconstruct_image_from_base64
from websocket.websocket_server import start

MODEL = "llava:7b" # model from Ollama
URL = "http://localhost:11434/api/chat" 
URI = "ws://localhost:8765" # websocket server URI
UI_DATA_QUEUE = Queue() # Queue to store data from the UI, fetched from synchronously

DRONE_COUNT = 5

USE_VLM = False



# message history, starts off with explaining what the model 
# will be doing in the search and rescue mission
message_history = [
    {
        'role': 'user',
        'content':  
            "You are a drone operator conducting a simulated search and rescue mission.\n" +
            f"You have {DRONE_COUNT} drone(s) at your disposal to locate a missing person.\n" +
            "Your primary task is to create and assign waypoints for the drones, analyze captured images, and determine the target's location."
        ,
    },
    {
        'role': 'assistant',
        'content': "Understood. I am ready to assist in the search and rescue mission.",
    },
    {
        'role': 'user',
        'content': (
            "The search process follows these steps:\n"
            "1. You will receive a list of waypoints with their locations, along with the current locations of the drones.\n"
            "2. Using this information, generate a target waypoint for each drone, utilizing the distance they are away and the waypoint's priority.\n"
            "3. The drones will travel to their individual waypoint and capture infrared (IR) images to detect potential heat signatures.\n"
            "4. If a heat signature is detected, the drones will take regular images to visually confirm the target.\n"
            "5. The drones will transmit regular images to you for analysis.\n"
            "6. Based on your analysis, determine whether the detected heat signature matches the missing person.\n"
            "7. If the target is confirmed, the mission is complete. If not, repeat the process until the target is found. You will need to assign a new waypoint if a drone has finished searching its current target.\n"
            "8. You may be asked to generate additional waypoints if all waypoints have been searched. You will be provided with a list of already searched locations."
            "Your objective is to accurately locate and confirm the missing person’s position using drone imagery.\n"
            "Now, let's begin by generating the initial target waypoints for each drone."
        ),
    },
    {
        'role': 'assistant',
        'content': "Please provide the initial waypoint list and wait while I generate the target waypoints for each drone.",
    }
]



async def connect_websocket():
    """Async function to connect to the websocket server"""
    async with websockets.connect(URI) as websocket:
        while True:
            msg = await websocket.recv()
            UI_DATA_QUEUE.put(msg)  # Store message for sync retrieval

def start_websocket_client():
    """Start the websocket client (on the simulation side) to connect to the server"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(connect_websocket())



def fetch_websocket_data():
    """Synchronous function to get data from the websocket queue from the UI"""
    if not UI_DATA_QUEUE.empty():
        return UI_DATA_QUEUE.get()
    return None  # No new message




def parentController(drone_count):
    manager = mp.Manager() # Manager to share data between processes


    if USE_VLM:
        print("Loading VLM...")
        load_model(MODEL)
    
    
    current_target_dictionary = manager.dict()  # Dictionary to store the current target waypoint of each drone
    status_dictionary = manager.dict() # Dictionary to store status of each drone
    target_found = mp.Value('b', False) # Global variable to stop the search loop when the target is found
    searched_areas_dictionary = manager.dict() # Dictionary mapping waypoint names to their locations that have been searched already
    image_queue = manager.Queue()  # Queue to store images waiting to be processed
    waypoint_queue = []
    




    # DOE1 is a deer in front of the spawn area (for testing)
    heapq.heappush(waypoint_queue, Waypoint("DOE1", 120, -50, -30, 3))


    processes = []  # List to store process references for each drone
    # Create and start processes for each drone
    for x in range(1):
        drone_name = str(x)
        current_target_dictionary[drone_name] = None # an instance of the waypoint class
        status_dictionary[drone_name] = "INITIALIZING"
        p = mp.Process(target=sdc.singleDroneController, args=(drone_name, current_target_dictionary, status_dictionary, target_found, searched_areas_dictionary, image_queue, waypoint_queue))
        p.start()
        processes.append(p)
        print(f"Drone {drone_name} is initializing")
    




    # Start the websocket server as a separate thread    
    websocket_server_thread = threading.Thread(target=start, daemon=True)
    websocket_server_thread.start()


    # Start the websocket client as a separate thread (on the simulation side)
    websocket_client_thread = threading.Thread(target=start_websocket_client, daemon=True)
    websocket_client_thread.start()

   
    while not all(status == "WAITING" for status in status_dictionary.values()):       
        print("Waiting for all drones to take off...")
        time.sleep(5)


    # Send the initial waypoints to the VLM
    if USE_VLM:
        message_history.append({
            'role': 'user',
            'content': "The waypoint queue is: " + str(waypoint_queue) + ". Only assign waypoints that are in the waypoint queue. The current target dictionary is: " + str(current_target_dictionary) + f". Please modify the current target dictionary and return it under assigned_target_dictionary. Set the current target of a drone by mapping the drone id (0 through {DRONE_COUNT} - 1) to the waypoint name."
        })
        response = chat(
            messages = message_history,
            model = MODEL,
            format = VLMOutput.model_json_schema()
        )
    
        print(response)
        message = VLMOutput.model_validate_json(response.message.content)
        print(message.assigned_target_dictionary)

        time.sleep(60)


    start_time = time.time()

    try:
        while not target_found.value:
            
            # Fetch WebSocket messages (sync retrieval)
            websocket_data = fetch_websocket_data()
            if websocket_data:
                print(f"Received UI WebSocket message: {websocket_data}")

            # only check drone status every 5 seconds
            if time.time() - start_time > 5:

                # assign waypoints to any waiting drones
                for drone_name in status_dictionary:
                    if status_dictionary[drone_name] == "WAITING":
                        if len(waypoint_queue) > 0:
                            next_waypoint = heapq.heappop(waypoint_queue)
                            current_target_dictionary[drone_name] = next_waypoint
                            print(f"Assigning waypoint {next_waypoint.name} to Drone {drone_name}")
                        else:
                            print(f"No waypoints available. Requesting new waypoints from VLM model...")
                        
                            # use the searched_areas dictionary to ask the VLM model for new waypoints (TODO)
                        
                
                start_time = time.time()




            # check for images waiting to be processed
            if not image_queue.empty():
                image = image_queue.get() # base64 image
                print(f"Received image from Drone {image.drone_id}")

                # reconstruct the image from the base64 string
                image_path = os.path.join(imgDir, f"image_{image.drone_id}.png")
                reconstruct_image_from_base64(image.image, image_path)

                # send the image to the VLM model for analysis
                if USE_VLM:
                    print("Not implemented yet")
                    # TODO



            


            time.sleep(1)  # Give some time between cycles
        
    except KeyboardInterrupt:
        for p in processes:
            p.terminate()

    # Wait for all processes to finish
    for p in processes:
        p.join()







if __name__ == "__main__":
    # directory to store pictures
    imgDir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'images')

    # create directory if it does not exist
    if not os.path.exists(imgDir):
        os.makedirs(imgDir)

    mp.set_start_method('spawn')

    parentController(DRONE_COUNT)
