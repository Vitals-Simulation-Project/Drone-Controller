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

# project imports
import local_config
import single_drone_controller as sdc
from model_files.pull_model import load_model
from classes import Waypoint, Image, VLMOutput
from helper_functions import reconstruct_image_from_base64
from websocket.websocket_server import start_websocket_server

MODEL = "llava:7b" # model from Ollama
URL = "http://localhost:11434/api/chat" 
URI = "ws://localhost:8765" # websocket server URI

DRONE_COUNT = 1

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





# Ensure 'spawn' is set at the beginning as the start method for multiprocessing
# This makes sure it will work on Windows
if __name__ == "__main__":
    mp.set_start_method("spawn", force=True)


# Directory to store pictures
imgDir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'images')

# Create directory if it does not exist
if not os.path.exists(imgDir):
    os.makedirs(imgDir)


# Starts a new thread for each drone and handles the setup
def setup_processes(drone_count):
    manager = mp.Manager()

    current_target_dictionary = manager.dict()
    status_dictionary = manager.dict()
    target_found = manager.Value("b", False)
    searched_areas_dictionary = manager.dict()
    image_queue = manager.Queue()
    waypoint_queue = []

    # DOE1 is a deer in front of spawn
    heapq.heappush(waypoint_queue, Waypoint("DOE1", 120, -50, -30, 3))

    processes = []
    for x in range(drone_count):
        drone_name = str(x)
        current_target_dictionary[drone_name] = None
        status_dictionary[drone_name] = "INITIALIZING"

        p = mp.Process(target=sdc.singleDroneController, args=(
            drone_name, current_target_dictionary, status_dictionary,
            target_found, searched_areas_dictionary, image_queue, waypoint_queue))
        p.start()
        processes.append(p)

        print(f"Drone {drone_name} is initializing")

    return processes, manager, current_target_dictionary, status_dictionary, target_found, image_queue, waypoint_queue


# Main async function to handle the program flow
async def parentController(drone_count):
    """Handles async parts of the program."""

    print("Waiting for drones to initialize...")
    # Setup multiprocessing in a separate thread (non-blocking)
    loop = asyncio.get_running_loop()
    processes, manager, current_target_dictionary, status_dictionary, target_found, image_queue, waypoint_queue = await loop.run_in_executor(
        None, setup_processes, drone_count
    )


    # Wait until all drones are in "WAITING" state
    while not all(status == "WAITING" for status in status_dictionary.values()):
        await asyncio.sleep(1)
    

    print("Starting the websocket server...", end="")
    # Start the WebSocket server as a background task
    websocket_task = asyncio.create_task(start_websocket_server())
    print("Done.")

    start_time = time.time()

    try:
        async with websockets.connect(URI) as websocket:

            while not target_found.value:


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


                await asyncio.sleep(1) # give some time between cycles

    except KeyboardInterrupt:
        for p in processes:
            p.terminate()

    for p in processes:
        p.join()

if __name__ == "__main__":
    asyncio.run(parentController(DRONE_COUNT))
