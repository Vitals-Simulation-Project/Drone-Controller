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
from websocket.websocket_server import start_websocket_server, send

MODEL = "gemma3:4b" # model from Ollama
URL = "http://localhost:11434/api/chat" 
URI = "ws://localhost:8765" # websocket server URI
RECEIVED_UI_DATA_QUEUE = Queue() # Queue to store data from the UI, fetched from synchronously
SEND_UI_DATA_QUEUE = Queue() # Queue to store data to be sent to the UI

WEBSOCKET_CLIENT = None # Global websocket client 


DRONE_COUNT = 5

TEST_VLM = True
RELEASE_BUILD = False



# message history, starts off with explaining what the model 
# will be doing in the search and rescue mission
# message_history = [
#     {
#         'role': 'user',
#         'content':  
#             "You are a drone operator conducting a simulated search and rescue mission.\n" +
#             f"You have {DRONE_COUNT} drone(s) at your disposal to locate a missing person.\n" +
#             "Your primary task is to create and assign waypoints for the drones, analyze captured images, and determine the target's location."
#         ,
#     },
#     {
#         'role': 'assistant',
#         'content': "Understood. I am ready to assist in the search and rescue mission.",
#     },
#     {
#         'role': 'user',
#         'content': (
#             "The search process follows these steps:\n"
#             "1. You will receive a list of waypoints with their locations, along with the current locations of the drones.\n"
#             "2. Using this information, generate a target waypoint for each drone, utilizing the distance they are away and the waypoint's priority.\n"
#             "3. The drones will travel to their individual waypoint and capture infrared (IR) images to detect potential heat signatures.\n"
#             "4. If a heat signature is detected, the drones will take regular images to visually confirm the target.\n"
#             "5. The drones will transmit regular images to you for analysis.\n"
#             "6. Based on your analysis, determine whether the detected heat signature matches the missing person.\n"
#             "7. If the target is confirmed, the mission is complete. If not, repeat the process until the target is found. You will need to assign a new waypoint if a drone has finished searching its current target.\n"
#             "8. You may be asked to generate additional waypoints if all waypoints have been searched. You will be provided with a list of already searched locations."
#             "Your objective is to accurately locate and confirm the missing person’s position using drone imagery.\n"
#             "Now, let's begin by generating the initial target waypoints for each drone."
#         ),
#     },
#     {
#         'role': 'assistant',
#         'content': "Please provide the initial waypoint list and wait while I generate the target waypoints for each drone.",
#     }
# ]

message_history = [
    {
        'role': 'user',
        'content': (  
            "You are a drone operator conducting a simulated search and rescue mission.\n" 
            "Your primary task is to create and assign waypoints for the drones, analyze captured images, and determine the target's location."
        )
    },
    {
        'role': 'assistant',
        'content': (
            "Understood. I am ready to assist in the search and rescue mission."                   
        )
    }
]




async def connect_websocket():
    """Async function to connect to the websocket server"""
    global WEBSOCKET_CLIENT
    async with websockets.connect(URI) as websocket:
        WEBSOCKET_CLIENT = websocket
        asyncio.create_task(send_message())
        while True:
            msg = await websocket.recv()
            RECEIVED_UI_DATA_QUEUE.put(msg)  # Store message for sync retrieval

async def send_message():
    """Async function to send a message to the websocket server"""
    while True:
        try:
            msg = SEND_UI_DATA_QUEUE.get_nowait()
            await WEBSOCKET_CLIENT.send(msg)
        except:
            await asyncio.sleep(1)

def send_to_ui(message):
    """Function to send a message to the UI via the websocket server"""
    SEND_UI_DATA_QUEUE.put(message)
            

def start_websocket_client():
    """Start the websocket client (on the simulation side) to connect to the server"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(connect_websocket())



def fetch_websocket_data():
    """Synchronous function to get data from the websocket queue from the UI"""
    if not RECEIVED_UI_DATA_QUEUE.empty():
        return RECEIVED_UI_DATA_QUEUE.get()
    return None




def parentController(drone_count):
    manager = mp.Manager() # Manager to share data between processes

    # Start the websocket server as a separate thread    
    websocket_server_thread = threading.Thread(target=start_websocket_server, daemon=True)
    websocket_server_thread.start()


    # Start the websocket client as a separate thread (on the simulation side)
    websocket_client_thread = threading.Thread(target=start_websocket_client, daemon=True)
    websocket_client_thread.start()


    # launch the unreal executable
    if RELEASE_BUILD:
        print("Launching project executable...")
        os.startfile(local_config.EXECUTABLE_PATH)
    else:
        print("Running in editor mode. No executable to launch.")
        


    # block and wait until start message is received from the UI



    time.sleep(5) # wait for the UI to connect
    # receive initial list of waypoints








    if TEST_VLM:
        print("Loading VLM...")
        load_model(MODEL)
    
    
    current_target_dictionary = manager.dict()   # Dictionary to store the current target waypoint of each drone
    status_dictionary = manager.dict()           # Dictionary to store status of each drone
    target_found = mp.Value('b', False)          # Global variable to stop the search loop when the target is found
    searched_areas_dictionary = manager.dict()   # Dictionary mapping waypoint names to their locations that have been searched already
    image_queue = manager.Queue()                # Queue to store images waiting to be processed
    waypoint_queue = []                          # Priority queue to store waypoints
    




    # DOE1 is a deer in front of the spawn area (for testing)
    #heapq.heappush(waypoint_queue, Waypoint("DOE1", 120, -50, 30, 3))



    processes = []  # List to store process references for each drone
    # Create and start processes for each drone
    for x in range(1):
        drone_name = str(x)
        current_target_dictionary[drone_name] = None # an instance of the waypoint class
        status_dictionary[drone_name] = "INITIALIZING"
        status_data = {
            "MessageType": "UpdateDroneState",
            "DroneID": int(x),
            "State": "INITIALIZING"
        }
        send_to_ui(json.dumps(status_data))
        p = mp.Process(target=sdc.singleDroneController, args=(drone_name, current_target_dictionary, status_dictionary, target_found, searched_areas_dictionary, image_queue, waypoint_queue))
        p.start()
        processes.append(p)
        print(f"Drone {drone_name} is initializing")
    






   
    while not all(status == "IDLE" for status in status_dictionary.values()):       
        print("Waiting for all drones to take off...")
        time.sleep(5)


    # Send the initial waypoints to the VLM
    # if TEST_VLM:
    #     message_history.append({
    #         'role': 'user',
    #         'content': "The waypoint queue is: " + str(waypoint_queue) + ". Only assign waypoints that are in the waypoint queue. The current target dictionary is: " + str(current_target_dictionary) + f". Please modify the current target dictionary and return it under assigned_target_dictionary. Set the current target of a drone by mapping the drone id (0 through {DRONE_COUNT} - 1) to the waypoint name."
    #     })
    #     response = chat(
    #         messages = message_history,
    #         model = MODEL,
    #         format = VLMOutput.model_json_schema()
    #     )
    
    #     print(response)
    #     message = VLMOutput.model_validate_json(response.message.content)
    #     print(message.assigned_target_dictionary)

    #     time.sleep(60)


    start_time = time.time()

    try:
        while not target_found.value:
            
            # fetch WebSocket messages (sync retrieval)
            websocket_data = fetch_websocket_data()
            if websocket_data:
                print(f"Received UI WebSocket message: {websocket_data}")

                # process the JSON data
                json_data = json.loads(websocket_data)
                if json_data["MessageType"] == "AddWaypoint":
                    # divide by 100 to convert from cm to m
                    waypoint = Waypoint(json_data["WaypointID"], json_data["X"] / 100, json_data["Y"] / 100, json_data["Z"] / 100, json_data["Priority"])
                    heapq.heappush(waypoint_queue, waypoint)
                    print(f"Added waypoint {waypoint.name} to the queue")
                elif json_data["MessageType"] == "DeleteWaypoint":
                    # delete the waypoint from the queue
                    for i, wp in enumerate(waypoint_queue):
                        if wp.name == json_data["WaypointID"]:
                            del waypoint_queue[i]
                            print(f"Deleted waypoint {wp.name} from the queue")
                            break
                    # fix the heap
                    heapq.heapify(waypoint_queue)
                else:
                    print("Unexpected message type")



            # assign waypoints to any waiting drones
            for drone_name in status_dictionary:
                if status_dictionary[drone_name] == "IDLE":
                    # if TEST_VLM and len(waypoint_queue) > 0:
                        # # ask the VLM model for the next waypoint to be assigned
                        # message_history.append({
                        #     'role': 'user',
                        #     'content': "The waypoint queue is: " + str(waypoint_queue) + ". Only assign waypoints that are in the waypoint queue. The current target dictionary is: " + str(current_target_dictionary) + f". Please modify the current target dictionary and return it under assigned_target_dictionary. Set the current target of a drone by mapping the drone id (0 through {DRONE_COUNT} - 1) to the waypoint name."
                        # })
                        # response = chat(
                        #     messages = message_history,
                        #     model = MODEL,
                        #     format = VLMOutput.model_json_schema()
                        # )
                    
                        # print(response)
                        # message = VLMOutput.model_validate_json(response.message.content)
                        # print(message.assigned_target_dictionary)

                        # assigned_target = message.assigned_target_dictionary[drone_name]
                        # current_target_dictionary[drone_name] = assigned_target
                        # print(f"Assigned waypoint {assigned_target} to Drone {drone_name}")
                        # print("Not implemented yet")




                    if len(waypoint_queue) > 0:
                        next_waypoint = heapq.heappop(waypoint_queue)
                        current_target_dictionary[drone_name] = next_waypoint
                        print(f"Assigning waypoint {next_waypoint.name} to Drone {drone_name}")
                    else:
                        print(f"No waypoints available. Requesting new waypoints from VLM model...")
                    
                        # use the searched_areas dictionary to ask the VLM model for new waypoints (TODO)
                        
                




            # check for images waiting to be processed
            if not image_queue.empty():
                image = image_queue.get() # base64 image
                print(f"Received image from Drone {image.drone_id}")

                # reconstruct the image from the base64 string || TODO: Don't save the image to disk here, but rather from the drone
                image_path = os.path.join(imgDir, f"drone_{image.drone_id}", f"waypoint_{image.waypoint_name}_{image.image_type}.png")
                reconstruct_image_from_base64(image.image, image_path)




                # send the image to the VLM model for analysis
                if TEST_VLM:
                    print("Sending image to VLM model for analysis...")
                    #print("image b64 ", image.image)
                    message_history.append({
                        'role': 'user',
                        'content': "Please analyze this image and determine if a human is present, set human_present_in_image to True if a human is present.",
                        'image': [image.image]
                    })
                    response = chat(
                        messages = message_history,
                        model = MODEL,
                        format = VLMOutput.model_json_schema()
                    )
                    # response = chat(
                    #     messages = {
                    #     'role': 'user',
                    #     'content': "Please analyze this image and set human_present_in_image to True if a human is present.",
                    #     'image': [image_path]
                    #     },
                    #     model = MODEL,
                    #     format = VLMOutput.model_json_schema()
                    # )
                
                    print(response)
                    message = VLMOutput.model_validate_json(response.message.content)

                    if message.human_present_in_image:
                        print(f"Drone {image.drone_id} has found a human")
                    




            # send updates to the websocket server (UI)
            # update drone targets
            for drone_name in current_target_dictionary:
                target = current_target_dictionary[drone_name]
                target_data = {
                    "MessageType": "UpdateDroneTarget",
                    "DroneID": int(drone_name),
                    "WaypointID": target.name if target else None
                }
                #print(f"Sending target update for Drone {drone_name} to UI")
                send_to_ui(json.dumps(target_data))



            # update drone states
            for drone_name in status_dictionary:
                status = status_dictionary[drone_name]
                status_data = {
                    "MessageType": "UpdateDroneState",
                    "DroneID": int(drone_name),
                    "State": status
                }
                #print(f"Sending status update for Drone {drone_name} to UI")
                send_to_ui(json.dumps(status_data))

            


            time.sleep(1)  # Give some time between cycles
        
    except KeyboardInterrupt:
        for p in processes:
            p.terminate()

        # Wait for all processes to finish
        for p in processes:
            p.join(timeout=3) # Wait for at most 3 seconds for each process to finish
        








if __name__ == "__main__":
    # directory to store pictures
    imgDir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'images')

    # create directory if it does not exist
    if not os.path.exists(imgDir):
        os.makedirs(imgDir)


    # create an image sub directory for each drone
    for i in range(DRONE_COUNT):
        droneDir = os.path.join(imgDir, f"drone_{i}")
        if not os.path.exists(droneDir):
            os.makedirs(droneDir)

    mp.set_start_method('spawn')

    parentController(DRONE_COUNT)
