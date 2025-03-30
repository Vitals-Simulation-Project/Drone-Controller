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


VLM_RESPONSE_LIST = {} # dictionary to store VLM responses, maps request number to the response
SEND_TO_VLM_QUEUE = Queue() # Queue to store requests to be sent to the VLM model

request_number = 1 # the next request number to be assigned


WEBSOCKET_CLIENT = None # Global websocket client 


DRONE_COUNT = 5

TEST_VLM = True # set to True if testing the VLM model, False if not
RELEASE_BUILD = False # set to True if running the executable, False if running in editor mode
VLMTIMEOUT = 600 # time out for VLM model in seconds





message_history = [
    {
        'role': 'user',
        'content': (  
            "You are a drone operator conducting a simulated search and rescue mission.\n" 
            "Your primary task is to create new waypoints for the drones, assign waypoints to drones, and analyze captured images."
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


def send_request_to_VLM(request):
    global request_number
    request_id = request_number # store the request number for the response
    request_number += 1 # increment the request number for the next request
    """Send a request to the VLM model and return the response in the queue"""
    message = [ 
        {
        'role': 'system',
        'content': (
            "You are a drone operator conducting a simulated search and rescue mission.\n" 
            "Your primary task is to create new waypoints for the drones, assign waypoints to drones, and analyze captured images."
        )
        },
        {
        'role': 'assistant',
        'content': (
            "Understood. I am ready to assist in the search and rescue mission."                   
        )
        },
        { 
        'role': 'user',
        'content': request,
        }
    ]
    response = chat(
        messages = message,
        model = MODEL,
        format = VLMOutput.model_json_schema()
    )
    response = VLMOutput.model_validate_json(response.message.content)
    VLM_RESPONSE_LIST[request_id] = response # Put the response in the queue for synchronous retrieval later

def start_VLM_thread():
    """Start the VLM thread to handle requests and responses"""
    while True:
        if not SEND_TO_VLM_QUEUE.empty():
            request = SEND_TO_VLM_QUEUE.get()
            print(f"Sending request ID {request_number} to VLM model")
            send_request_to_VLM(request)
        time.sleep(1)  # Give some time between cycles


def fetch_VLM_response(request_id):
    """Synchronous function to get data from the VLM response queue"""
    global VLM_RESPONSE_LIST
    if request_id in VLM_RESPONSE_LIST:
        response = VLM_RESPONSE_LIST[request_id]
        del VLM_RESPONSE_LIST[request_id]
        return response
    return None






# main loop
def parentController():
    manager = mp.Manager() # manager to share data between processes

    current_target_dictionary = manager.dict()   # dictionary to store the current target waypoint of each drone
    current_position_dictionary = manager.dict() # dictionary to store the current position of each drone
    status_dictionary = manager.dict()           # dictionary to store status of each drone
    request_dictionary = {}                      # dictionary that maps drone names to the ID of their request
    target_found = mp.Value('b', False)          # global variable to stop the search loop when the target is found
    searched_areas_dictionary = manager.dict()   # dictionary mapping waypoint names to their locations that have been searched already
    image_queue = manager.Queue()                # queue to store images waiting to be processed
    waypoint_queue = []                          # priority queue to store waypoints



    # start the websocket server as a separate thread    
    websocket_server_thread = threading.Thread(target=start_websocket_server, daemon=True)
    websocket_server_thread.start()

    # start the websocket client as a separate thread (on the simulation side)
    websocket_client_thread = threading.Thread(target=start_websocket_client, daemon=True)
    websocket_client_thread.start()

    # start the VLM thread to handle requests and responses
    VLM_thread = threading.Thread(target=start_VLM_thread, daemon=True)
    VLM_thread.start()



    # launch the unreal executable
    if RELEASE_BUILD:
        print("Launching project executable...")
        os.startfile(local_config.EXECUTABLE_PATH)
    else:
        print("Running in editor mode. No executable to launch.")
        

    time.sleep(5) # wait for the executable to launch and websocket to launch


    # block and wait until start message is received from the UI
    print("Waiting for start message from UI...", end="")
    while True:
        websocket_data = fetch_websocket_data()
        if websocket_data:
            #print(f"Received UI WebSocket message: {websocket_data}")
            json_data = json.loads(websocket_data)
            if json_data["MessageType"] == "StartSimulation":
                break
            elif json_data["MessageType"] == "StopSimulation":
                print("Stopping simulation...")
                target_found.value = True
                exit(0)
            else:
                print("Unexpected startup message, exiting...")
                exit(0)    
        time.sleep(1)
    
    print("Received!")

    time.sleep(5) # wait for the initial waypoints to be sent

    # receive initial list of waypoints
    while True:
        websocket_data = fetch_websocket_data()
        if websocket_data:
            #print(f"Received UI WebSocket message: {websocket_data}")
            json_data = json.loads(websocket_data)
            if json_data["MessageType"] == "AddWaypoint":
                # divide by 100 to convert from cm to m
                waypoint = Waypoint(json_data["WaypointID"], json_data["X"] / 100, json_data["Y"] / 100, json_data["Z"] / 100, json_data["Priority"])
                heapq.heappush(waypoint_queue, waypoint)
                print(f"Added waypoint {waypoint.name} to the queue")
        else:
            print("Done!")
            break







    if TEST_VLM:
        print("Loading VLM...")
        load_model(MODEL)
    
    
    # TODO: Send initial chat message to the VLM model to explain the mission



    processes = []  # list to store process references for each drone
    # Create and start processes for each drone
    for x in range(DRONE_COUNT):
        drone_name = str(x)
        current_target_dictionary[drone_name] = None # an instance of the waypoint class
        status_dictionary[drone_name] = "INITIALIZING"
        request_dictionary[drone_name] = None
        current_position_dictionary[drone_name] = None # only used for assigning new waypoints to drones
        status_data = {
            "MessageType": "UpdateDroneState",
            "DroneID": int(x),
            "State": "INITIALIZING"
        }
        send_to_ui(json.dumps(status_data))
        p = mp.Process(target=sdc.singleDroneController, args=(drone_name, current_target_dictionary, status_dictionary, target_found, searched_areas_dictionary, image_queue, waypoint_queue, current_position_dictionary))
        p.start()
        processes.append(p)
        print(f"Drone {drone_name} is initializing")
    

   
    while not all(status == "IDLE" for status in status_dictionary.values()):       
        print("Waiting for all drones to take off...")
        time.sleep(5)
    
    # send to ui that all drones are ready
    for drone_name in status_dictionary:
        status_data = {
            "MessageType": "UpdateDroneState",
            "DroneID": int(drone_name),
            "State": status_dictionary[drone_name]
        }
        send_to_ui(json.dumps(status_data))





    # give each drone their initial target waypoint from the VLM
    if TEST_VLM and len(waypoint_queue) >= DRONE_COUNT:
        # change drone state to be waiting for waypoint to be assigned
        for drone_name in status_dictionary:
            status_dictionary[drone_name] = "WAITING"
            status_data = {
                "MessageType": "UpdateDroneState",
                "DroneID": int(drone_name),
                "State": status_dictionary[drone_name]
            }
            send_to_ui(json.dumps(status_data))


        # ask the VLM model for the next waypoint to be assigned
        waypoint_queue_str = ", ".join([f"{wp.name} ({wp.x}, {wp.y}, {wp.z})" for wp in waypoint_queue])
        position_str = ", ".join([f"{drone_name} ({current_position_dictionary[drone_name][0]}, {current_position_dictionary[drone_name][1]}, {current_position_dictionary[drone_name][2]})" for drone_name in current_position_dictionary])                    
        # request = (
        #     f"Please assign an initial target waypoint to each drone. "
        #     f"There are {DRONE_COUNT} drones. "
        #     f"The waypoint queue is: {waypoint_queue_str}. "
        #     f"Each waypoint has its ID and coordinates in parentheses. "
        #     f"Only assign waypoints that are in the waypoint queue. "
        #     f"Please return your response under assigned_target_dictionary. "
        #     f"Set the current target of a drone by mapping the drone ID (a number) "
        #     f"to only the waypoint ID (a number). "
        #     f"Try to assign drones to waypoints that are closest to them. "
        #     f"The current positions of the drones for which you should assign a position is: {position_str}, "
        #     f"where each drone has its ID and coordinates in parentheses. "
        #     f"Do not assign to drones that were not mentioned in the list of current positions."
        # )
        request = (
            f"Assign an initial waypoint to each drone. "
            f"There are {DRONE_COUNT} drones. The available waypoints in the format 'waypoint name' (coordinates) are: {waypoint_queue_str}. "
            f"Each waypoint must come from this list. " 
            f"Only assign each waypoint to a single drone. Do NOT assign the same waypoint to multiple drones. "
            f"Assign waypoints to drones closest to them using their current positions 'drone_name' (coordinates): {position_str}. "
            f"Only assign waypoints to drones listed in the current positions. "
            f"Return the assignments as 'assigned_target_dictionary' in the format: {{drone_id: waypoint_id}} where drone_id and waypoint_id are both integers. "
            f"Each waypoint_id must be a unique integer and you must assign to each drone. "
            # f"If there are not enough drones, assign the first {DRONE_COUNT} waypoints to the drones. "
        )

        print(f"Sending request to VLM model: {request}")         
        SEND_TO_VLM_QUEUE.put(request)
        print(f"Request number {request_number} sent")
    elif len(waypoint_queue) >= DRONE_COUNT: 
        # assign the first DRONE_COUNT waypoints to the drones
        print("Assigning waypoints directly...")
        for i in range(DRONE_COUNT):
            assigned_target = heapq.heappop(waypoint_queue)
            current_target_dictionary[str(i)] = assigned_target
            print(f"Assigning waypoint {assigned_target.name} to Drone {i}")
    else:
        print("Not enough waypoints in the queue to assign using the VLM model. Assigning waypoints directly...")
        for i in range(len(waypoint_queue)):
            assigned_target = heapq.heappop(waypoint_queue)
            current_target_dictionary[str(i)] = assigned_target
            print(f"Assigning waypoint {assigned_target.name} to Drone {i}")
        

    # wait until the VLM responds
    while True:
        response = fetch_VLM_response(request_number - 1)
        if response is not None:
            break
        time.sleep(1)
    
    
    print(response.assigned_target_dictionary)

    for drone_name in response.assigned_target_dictionary:
        print(f"Checking drone {drone_name} for assigned target {response.assigned_target_dictionary[drone_name]}")
        if drone_name not in current_target_dictionary:
            print(f"VLM Error: There is no drone with the ID: {drone_name}")
            continue

        try:
            assigned_target = int(response.assigned_target_dictionary[drone_name])
        except ValueError:
            print(f"VLM Error: Assigned target {response.assigned_target_dictionary[drone_name]} is not a number")
            continue
        for i, wp in enumerate(waypoint_queue):
            if wp.name == assigned_target:
                assigned_target = wp
                del waypoint_queue[i]
                print(f"Deleted waypoint {assigned_target.name} from the queue")

                # fix the heap
                heapq.heapify(waypoint_queue)

                # assign the target to the drone
                current_target_dictionary[drone_name] = assigned_target
                print(f"The VLM has assigned waypoint {assigned_target.name} to Drone {drone_name}")
                break
        else: # backup plan if the VLM hallucinates
            print(f"VLM Error: Waypoint {assigned_target} not found in the waypoint queue. Assigning next waypoint from queue...")
            if len(waypoint_queue) > 0:
                # assign the next waypoint from the queue
                assigned_target = heapq.heappop(waypoint_queue)
                current_target_dictionary[drone_name] = assigned_target
                print(f"Assigning waypoint {assigned_target.name} to Drone {drone_name}")





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
                elif json_data["MessageType"] == "StopSimulation":
                    print("Stopping simulation...")
                    target_found.value = True # to stop the drones
                    break                
                else:
                    print("Unexpected message type")



            # assign waypoints to any waiting drones
            for drone_name in status_dictionary:
                #print(f"The status of Drone {drone_name} is {status_dictionary[drone_name]}")
                if TEST_VLM and len(waypoint_queue) > 0 and status_dictionary[drone_name] == "IDLE":
                    # change drone state to be waiting for waypoint to be assigned
                    status_dictionary[drone_name] = "WAITING"


                    # ask the VLM model for the next waypoint to be assigned
                    waypoint_queue_str = ", ".join([f"waypoint: {wp.name} ({wp.x}, {wp.y}, {wp.z})" for wp in waypoint_queue])                    
                    request = f"Please assign a target waypoint to the drone with ID: {drone_name}. The waypoint queue is: {waypoint_queue_str}. Each waypoint has it's ID and coordinates in parentheses. Only assign waypoints that are in the waypoint queue. Please return your response under assigned_target_dictionary. Set the current target of a drone by mapping the drone ID to only the waypoint ID (a number). Try to assign drones to waypoints that are closest to them. The current position of this drone is: {current_position_dictionary[drone_name]}."
                    print(f"Sending request to VLM model: {request}")         
                    SEND_TO_VLM_QUEUE.put(request)

                    # add the request id to the request dictionary
                    request_dictionary[drone_name] = request_number # store the request number for the response
                    print(f"Request number {request_number} assigned to Drone {drone_name}")
                                    

                elif TEST_VLM and status_dictionary[drone_name] == "WAITING":
                    # get the response from the VLM model
                    response = fetch_VLM_response(request_dictionary[drone_name])
                    if response is None:
                        continue



                    
                    print(response.assigned_target_dictionary)

                    assigned_target = int(response.assigned_target_dictionary[drone_name])
                    print(f"Assigned target for Drone {drone_name}: {assigned_target}")
                    for wp in waypoint_queue:
                        if wp.name == assigned_target:
                            assigned_target = wp
                            break
                    else:
                        assigned_target = None


                    if assigned_target is not None:
                        # remove the assigned target from the waypoint queue
                        for i, wp in enumerate(waypoint_queue):
                            if wp.name == assigned_target.name:
                                del waypoint_queue[i]
                                print(f"Deleted waypoint {wp.name} from the queue")
                                break
                        # fix the heap
                        heapq.heapify(waypoint_queue)
                        # assign the target to the drone
                        current_target_dictionary[drone_name] = assigned_target
                        print(f"The VLM has assigned waypoint {assigned_target} to Drone {drone_name}")
                    else:
                        print(f"Waypoint {assigned_target} not found in the waypoint queue. Assigning next waypoint from queue...")
                        # assign the next waypoint from the queue
                        if len(waypoint_queue) > 0:
                            next_waypoint = heapq.heappop(waypoint_queue)
                            current_target_dictionary[drone_name] = next_waypoint
                            print(f"Assigning waypoint {next_waypoint.name} to Drone {drone_name}")
                        else:
                            print(f"No waypoints available. Requesting new waypoints from VLM model...")







                if len(waypoint_queue) > 0 and not TEST_VLM:
                    next_waypoint = heapq.heappop(waypoint_queue)
                    current_target_dictionary[drone_name] = next_waypoint
                    print(f"FALLBACK: Assigning waypoint {next_waypoint.name} to Drone {drone_name}")
                
                if len(waypoint_queue) == 0 and TEST_VLM:
                    #print(f"No waypoints available. Requesting new waypoints from VLM model...")
                    pass
                    # use the searched_areas dictionary to ask the VLM model for new waypoints (TODO)
                        
                




            # check for images waiting to be processed
            if not image_queue.empty():
                image = image_queue.get() # base64 image
                print(f"Received image from Drone {image.drone_id}")

                # reconstruct the image from the base64 string || TODO: Don't save the image to disk here, but rather from the drone
                image_path = os.path.join(imgDir, f"drone_{image.drone_id}", f"waypoint_{image.waypoint_name}_{image.image_type}.png")
                reconstruct_image_from_base64(image.image, image_path)




                # send the image to the VLM model for analysis
                if TEST_VLM and False:
                    print("Sending image to VLM model for analysis...")
                    #print("image b64 ", image.image)
                    message_history.append({
                        'role': 'user',
                        'content': "Please analyze this image and determine if a human is present, set human_present_in_image to True if a human is present.",
                        'images': [image.image],
                    })
                    try:
                        # response = chat(
                        #     messages = message_history,
                        #     model = MODEL,
                        #     format = VLMOutput.model_json_schema(),
                        #     timeout=5
                        # )
                        # response = chat(
                        #     messages = {
                        #     'role': 'user',
                        #     'content': "Please analyze this image and set human_present_in_image to True if a human is present.",
                        #     'image': [image_path]
                        #     },
                        #     model = MODEL,
                        #     format = VLMOutput.model_json_schema()
                        # )

                        data = {
                            "model": MODEL,
                            "messages": [
                                {
                                    "role": "user",
                                    "content": "Please analyze this image and set human_present_in_image to True if a human is present.",
                                    "images": [image.image]
                                }
                            ],
                            "stream": False
                        }

                        response = requests.post(URL, json=data, timeout=VLMTIMEOUT)
                        response = json.loads(response.text)
                        
                        
                    
                        print(response)
                        # message = VLMOutput.model_validate_json(response.message.content)

                        # if message.human_present_in_image:
                        #     print(f"Drone {image.drone_id} has found a human")
                    except Exception as e:
                        print(f"Error: {e}")
                        print("VLM model timed out. Skipping image analysis.")
                    




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
        print("KeyboardInterrupt detected. Shutting down...")

    finally:
        for p in processes:
            p.terminate()  # Ensure all processes are terminated

        for p in processes:
            p.join(timeout=3)  # Wait for up to 3 seconds for each process to finish
        





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

    mp.set_start_method('spawn') # set the start method to spawn for multiprocessing, spawn is used for Windows

    # start the mission
    parentController()
