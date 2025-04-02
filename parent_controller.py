#=========================
#  parent_controller.py  #
#=========================

# Parent Process that manages communication between VLM, UI, and Drones

# Python Dependencies
import asyncio
import heapq
import json
import multiprocessing as mp
from ollama import chat # type: ignore
import os
from queue import Queue
import requests
import threading
import time
import websockets
import geopy.distance # type: ignore
import cv2

# Project Imports
from classes import Image, VLMOutput, Waypoint
from helper_functions import reconstruct_image_from_base64
import local_config
from model_files.pull_model import load_model
import single_drone_controller as sdc

# Developer Settings
TEST_VLM = True       # set to True if testing with the VLM model, False if not
RELEASE_BUILD = True  # set to True if running the executable, False if running in editor mode
VLM_TIMEOUT = 600     # time out for VLM model in seconds

# AI Constants
MODEL = "gemma3:4b"
CHAT_API_URL = "http://localhost:11434/api/chat"

VLM_RESPONSE_DICTIONARY = {} # dictionary to store VLM responses, maps request number to the response
SEND_TO_VLM_QUEUE = Queue() # Queue to store requests to be sent to the VLM model
request_number = 1 # the next request number to be assigned

# UI Constants
WEBSOCKET_SERVER_URI = "ws://localhost:8765"
WEBSOCKET_CLIENT = None
RECEIVED_UI_DATA_QUEUE = Queue() # Queue to store data receieved by the UI
SEND_UI_DATA_QUEUE = Queue() # Queue to store data to be sent to the UI

# Global Variables Set on Start Parent Controller
DRONE_COUNT = None
current_target_dictionary = None    # dictionary to store the current target waypoint of each drone
current_position_dictionary = None  # dictionary to store the current position of each drone
status_dictionary = None            # dictionary to store status of each drone
request_dictionary = None           # dictionary that maps drone names to the ID of their request
requeued_waypoints_list = None      # list of waypoints that have been requeued by a drone
target_found = None                 # global variable to stop the search loop when the target is found
searched_areas_dictionary = None    # dictionary mapping waypoint names to their locations that have been searched already
image_queue = None                  # queue to store images waiting to be processed
waypoint_queue = None               # priority queue to store waypoints
img_dir = None                      # directory to store images
all_waypoints_dictionary = None     # dictionary to store all waypoints in the simulation, including deleted and searched ones

# Keep processes and threads global to avoid early shutdown
drone_controller_processes = []
websocket_client_thread = None
VLM_thread = None

# Shutdown Event
SHUTDOWN_EVENT = None

message_history = [
    {
        'role': 'user',
        'content': (
            "You are a drone operator conducting a simulated search and rescue operation.\n"
            "Your only task is to analyze incoming images to determine if a person is present.\n"
            "You must not speculate or guess, there needs to be a clear reason why you believe a person is present."
        )
    }
]

def initialize(drone_count: int, shutdown_event):
    '''Initialize Global Variables'''

    manager = mp.Manager()

    global current_target_dictionary
    current_target_dictionary = manager.dict()

    global current_position_dictionary
    current_position_dictionary = manager.dict()

    global status_dictionary
    status_dictionary = manager.dict()

    global request_dictionary
    request_dictionary = {}

    global target_found
    target_found = mp.Value('b', False)

    global searched_areas_dictionary
    searched_areas_dictionary = manager.dict()

    global requeued_waypoints_list
    requeued_waypoints_list = manager.list()

    global all_waypoints_dictionary
    all_waypoints_dictionary = manager.dict()

    global image_queue
    image_queue = manager.Queue()

    global waypoint_queue
    waypoint_queue = []

    global DRONE_COUNT
    DRONE_COUNT = drone_count

    global SHUTDOWN_EVENT
    SHUTDOWN_EVENT = shutdown_event

def perform_startup_sequence():
    '''Start websocket and VLM threads, pull the model, and launch executable'''

    # start the websocket client as a separate thread (on the simulation side)
    global websocket_client_thread
    websocket_client_thread = threading.Thread(target=start_websocket_client_thread, daemon=True)
    websocket_client_thread.start()

    # start the VLM thread to handle requests and responses
    global VLM_thread
    VLM_thread = threading.Thread(target=start_VLM_thread, daemon=True)
    VLM_thread.start()

    time.sleep(5) # wait for threads to start

    # Load the VLM model
    if TEST_VLM:
        print("Loading VLM...")
        load_model(MODEL)

    # Launch Executable
    if RELEASE_BUILD:
        print("Launching Project Executable...")
        os.startfile(local_config.EXECUTABLE_PATH)
    else:
        print("Running in editor mode. No executable to launch.")


    # Block until user selects Difficulty and begins simulation
    print("\nSelect a Difficulty in UI to begin...")
    while True:
        websocket_data = fetch_websocket_data()
        if websocket_data:
            json_data = json.loads(websocket_data)

            if json_data["MessageType"] == "StartSimulation":
                break

            elif json_data["MessageType"] == "StopSimulation":
                print("Stopping simulation...")
                exit(0)

            else:
                print("Unexpected startup message, exiting...")
                exit(0) 

    print("Beginning Simulation...\n")

def receive_initial_waypoints():
    '''Receive Initial Waypoints sent from UI on Select Difficulty'''

    while True:
        time.sleep(.2) # Give time for websocket data to send
        websocket_data = fetch_websocket_data()
        if websocket_data:
            json_data = json.loads(websocket_data)
            if json_data["MessageType"] == "AddWaypoint":
                # divide by 100 to convert from cm to m
                waypoint = Waypoint(json_data["WaypointID"], json_data["X"] / 100, json_data["Y"] / 100, json_data["Z"] / 100, json_data["Priority"])
                heapq.heappush(waypoint_queue, waypoint)
                print(f"Added Waypoint {waypoint.name} to the queue")
        else:
            print("Done!")
            break

def initialize_drone_controller(id):
    '''Set Initial Status and Start Single Drone Controller Process'''

    global drone_controller_processes

    drone_name = str(id)
    current_target_dictionary[drone_name] = None # an instance of the waypoint class
    status_dictionary[drone_name] = None 
    request_dictionary[drone_name] = []
    current_position_dictionary[drone_name] = None

    drone_state_message = {
        "MessageType": "UpdateDroneState",
        "DroneID": id,
        "State": "INITIALIZING"
    }

    send_to_ui(json.dumps(drone_state_message))

    process = mp.Process(target=sdc.singleDroneController, args=(drone_name, current_target_dictionary, status_dictionary, target_found, searched_areas_dictionary, image_queue, waypoint_queue, current_position_dictionary, SHUTDOWN_EVENT, requeued_waypoints_list, all_waypoints_dictionary), daemon=True)
    process.start()
    drone_controller_processes.append(process)

    print(f"[Drone {drone_name}] Initializing")




def send_to_ui(message):
    """Function to send a message to the UI via the websocket server"""
    SEND_UI_DATA_QUEUE.put(message)  

def send_status_update():
    for drone_name in status_dictionary:
        status_data = {
            "MessageType": "UpdateDroneState",
            "DroneID": int(drone_name),
            "State": status_dictionary[drone_name]
        }
        send_to_ui(json.dumps(status_data))

def send_target_update():
    for drone_name in current_target_dictionary:
        target = current_target_dictionary[drone_name]
        target_data = {
            "MessageType": "UpdateDroneTarget",
            "DroneID": int(drone_name),
            "WaypointID": target.name if target else None
        }
        send_to_ui(json.dumps(target_data))

def assign_waypoints():
    '''If drone is WAITING, assign a waypoint from queue'''

    global requeued_waypoints_list
    global waypoint_queue
    # update queue with the waypoints waiting to be requeued
    for index, waypoint in enumerate(requeued_waypoints_list):
        heapq.heappush(waypoint_queue, waypoint)
        print(f"[Parent] Requeued waypoint {waypoint.name} to the queue")
        del requeued_waypoints_list[index] # remove the waypoint from the list


    

        

    # peek at the first waypoint to see if it's user added
    if len(waypoint_queue) > 0:        
        next_waypoint = waypoint_queue[0]
        #print(f"[Parent] Next Waypoint: {next_waypoint.name} with priority {next_waypoint.priority}")
        if next_waypoint.priority == 1:
            print(f"[Parent] User added waypoint {next_waypoint.name} to the queue")

            # find the closest drone to the waypoint
            closest_drone = None
            closest_distance = float("inf")
            for drone_name in current_position_dictionary:
                drone_position = current_position_dictionary[drone_name]
                if drone_position and status_dictionary[drone_name] != "SEARCHING" and current_target_dictionary[drone_name] and current_target_dictionary[drone_name].priority > 1:
                    distance = (drone_position[0] - next_waypoint.x) ** 2 + (drone_position[1] - next_waypoint.y) ** 2 + (drone_position[2] - next_waypoint.z) ** 2
                    distance = distance ** 0.5 # euclidean distance
                    if distance < closest_distance:
                        closest_distance = distance
                        closest_drone = drone_name
            
            print(f"The closest drone to waypoint {next_waypoint.name} is {closest_drone} with a distance of {closest_distance}m")
            if closest_drone:
                current_target_dictionary[closest_drone] = next_waypoint
                status_dictionary[closest_drone] = "MOVING"
                print(f"[Drone {closest_drone}] Reassigned to waypoint: {next_waypoint.name}")
                del waypoint_queue[0] # remove the waypoint from the queue
                heapq.heapify(waypoint_queue) # fix the heap


    for drone_name in status_dictionary:    
        if status_dictionary[drone_name] == "WAITING" or status_dictionary[drone_name] == "IDLE" or current_target_dictionary[drone_name] == None:
            if len(waypoint_queue) > 0:
                next_waypoint = heapq.heappop(waypoint_queue)
                current_target_dictionary[drone_name] = next_waypoint
                status_dictionary[drone_name] = "MOVING"
                print(f"[Drone {drone_name}] Assigned waypoint: {next_waypoint.name}")

def delete_searched_waypoints():
    global searched_areas_dictionary

    for waypoint in searched_areas_dictionary:
        delete_message = {
            "MessageType": "DeleteWaypoint",
            "ID": waypoint
        }

        send_to_ui(json.dumps(delete_message))
    
    searched_areas_dictionary.clear()


def fetch_VLM_response(request_ids):
    """Synchronous function to get data from the VLM response queue"""
    global VLM_RESPONSE_DICTIONARY
    response = []
    for request_id in request_ids:
        if request_id in VLM_RESPONSE_DICTIONARY:
            response.append(VLM_RESPONSE_DICTIONARY[request_id])
            del VLM_RESPONSE_DICTIONARY[request_id]
    return response if len(response) > 0 else None


# def send_request_to_VLM(request, drone_id, waypoint_name, image=None):
#     """Send a request to the VLM model and return the response in the queue"""

#     message = [ 
#         # {
#         # 'role': 'system',
#         # 'content': (
#         #     "You are a drone operator conducting a simulated search and rescue mission.\n" 
#         # )
#         # },
#         # {
#         # 'role': 'assistant',
#         # 'content': (
#         #     "Understood. I am ready to assist in the search and rescue mission."                   
#         # )
#         # },
#         { 
#         'role': 'user',
#         'content': request,
#         'images': [image] if image else None
#         }
#     ]
#     response = chat(
#         messages = message,
#         model = MODEL,
#         format = VLMOutput.model_json_schema()
#     )
#     print(f"[Parent] VLM Response: {response}")
#     try:
#         response = VLMOutput.model_validate_json(response.message.content)
#     except Exception as e:
#         print(f"[Parent] Error validating VLM response: {e}")
#         response = None 


#     # convert response from class to dictionary
#     if response:
#         response = vars(response)
#         response["waypoint_name"] = waypoint_name
    
#     print(f"[Parent] VLM Response as dictionary: {response}")

#     VLM_RESPONSE_DICTIONARY[drone_id] = response # Put the response in the queue for synchronous retrieval later (TODO: see if this breaks when multiple responses come in)

def send_request_to_VLM(request, drone_id, waypoint_name, image=None):
    """Send a request to the VLM model and return the response in the queue"""

    start_time = time.time()
    data = {
        "model": MODEL,
        "messages": [
            {
                "role": "user",
                "content": request,
                "images": [image] if image else None
            }
        ],
        "stream": False,
        "temperature": 0.1,
        "format": {
            "type": "object",
            "properties": {
                "human_present_in_image": {
                    "type": "boolean",
                    "description": "true if human is present in image, false otherwise"
                },
            },
            "required": ["human_present_in_image"]
        }
    }
    response = requests.post(CHAT_API_URL, json=data, timeout=VLM_TIMEOUT)
    if response.status_code != 200:
        print(f"[Parent] Error: {response.status_code} - {response.text}")
        print(f"[Parent] Image from drone {drone_id} at waypoint {waypoint_name} requires manual review.")
        VLM_RESPONSE_DICTIONARY[drone_id] = None
        return
    
    response = response.json()
    print(f"[Parent] Raw VLM Response: {response} with type {type(response)}")



    # construct the response dictionary from the response
    try:
        content = json.loads(response["message"]["content"])
        response_data = {
            "human_present_in_image": content["human_present_in_image"],
            "waypoint_name": waypoint_name,
            "response_time": time.time() - start_time
        }
    except Exception as e:
        print(f"[Parent] Error extracting VLM response: {e}")
        VLM_RESPONSE_DICTIONARY[drone_id] = None
        return

    
    print(f"[Parent] VLM Response as dictionary: {response_data}")

    VLM_RESPONSE_DICTIONARY[drone_id] = response_data # Put the response in the queue for synchronous retrieval later (TODO: see if this breaks when multiple responses come in)



def start_VLM_thread():
    """Start the VLM thread to handle requests and responses"""

    while True:
        if not SEND_TO_VLM_QUEUE.empty():
            request = SEND_TO_VLM_QUEUE.get()
            if len(request) > 3: # for image requests
                print(f"[Parent] Image request sent by drone {request[1]} at waypoint {request[2]}")
                send_request_to_VLM(request[0], request[1], request[2], request[3])
            else: # for assigning waypoints
                send_request_to_VLM(request[0], request[1], request[2], None)
        time.sleep(1)  # Give some time between cycles

async def send_message():
    """Async function to send a message to the websocket server"""

    while True:
        try:
            msg = SEND_UI_DATA_QUEUE.get_nowait()
            await WEBSOCKET_CLIENT.send(msg)
        except:
            await asyncio.sleep(1)

async def connect_to_websocket_server():
    """Async function to connect to the websocket server"""

    global WEBSOCKET_CLIENT
    async with websockets.connect(WEBSOCKET_SERVER_URI) as websocket:
        WEBSOCKET_CLIENT = websocket
        asyncio.create_task(send_message())
        while True:
            msg = await websocket.recv()
            RECEIVED_UI_DATA_QUEUE.put(msg)  # Store message for sync retrieval

def start_websocket_client_thread():
    """Start the websocket client (on the simulation side) to connect to the server"""

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(connect_to_websocket_server())

def fetch_websocket_data():
    """Synchronous function to get data from the websocket queue from the UI"""
    if not RECEIVED_UI_DATA_QUEUE.empty():
        return RECEIVED_UI_DATA_QUEUE.get()
    return None

def process_websocket_message(websocket_data):
    '''Handle websocket message sent from UI'''

    global waypoint_queue
    global current_target_dictionary
    global status_dictionary

    if websocket_data:
        
        json_data = json.loads(websocket_data)

        if json_data["MessageType"] == "AddWaypoint":

            # divide by 100 to convert from cm to m
            waypoint = Waypoint(json_data["WaypointID"], json_data["X"] / 100, json_data["Y"] / 100, json_data["Z"] / 100, json_data["Priority"])
            heapq.heappush(waypoint_queue, waypoint)
            print(f"\n[ADD OVERRIDE] Added waypoint {waypoint.name} to the queue with priority {waypoint.priority}\n")

        elif json_data["MessageType"] == "DeleteWaypoint":
            print(f"\n[DELETE] Deleting waypoint {json_data['WaypointID']}\n")
            # delete the waypoint from the queue
            for i, wp in enumerate(waypoint_queue):
                if wp.name == json_data["WaypointID"]:
                    del waypoint_queue[i]
                    print(f"\n[DELETE OVERRIDE] Deleted waypoint {wp.name} from the queue\n")
                    break

            for drone_name in current_target_dictionary:
                if current_target_dictionary[drone_name] and current_target_dictionary[drone_name].name == json_data["WaypointID"]:
                    current_target_dictionary[drone_name] = None
                    time.sleep(3)
                    status_dictionary[drone_name] = "IDLE"
                    print(f"[PARENT] Drone {drone_name} had its current target: {json_data['WaypointID']} deleted")

            # fix the heap
            heapq.heapify(waypoint_queue)

        elif json_data["MessageType"] == "StopSimulation":
            print("Stopping simulation...")
            SHUTDOWN_EVENT.set()               
        else:
            print("Unexpected message type")

def process_image_queue():
    '''Handle the queue of images to send to the VLM'''

    if not image_queue.empty():
        image = image_queue.get() # base64 image
        print(f"[Parent] Received Image from Drone {image.drone_id}")

        # reconstruct the image from the base64 string || TODO: Don't save the image to disk here, but rather from the drone
        image_path = os.path.join(img_dir, f"drone_{image.drone_id}", f"waypoint_{image.waypoint_name}_{image.image_type}.png")
        reconstruct_image_from_base64(image.image, image_path)


        if TEST_VLM:
            request = f"Analyze this image and determine if a human is present. Do not speculate or guess, you must be certain that there is a human. If you are certain that a human is present, set human_present_in_image to true. Otherwise set human_present_in_image to false."
            
            #request_dictionary[image.drone_id].append(request_number) # store the request number for the response
            SEND_TO_VLM_QUEUE.put((request, image.drone_id, image.waypoint_name, image.image))

            print(f"[Parent] Drone {image.drone_id} sent image to VLM model for analysis")
    

def process_vlm_response():
    '''Handle the VLM response from the queue'''

    for drone_name in VLM_RESPONSE_DICTIONARY:
        response = VLM_RESPONSE_DICTIONARY[drone_name]

        if response:
            # check if the response says there is a human present or not
            if response["human_present_in_image"]:
                print("\n[Parent] Drone {drone_name} found a human in the image taken at waypoint {response['waypoint_name']}")

                # open the image in a new window to show the user
                image_path = os.path.join(img_dir, f"drone_{drone_name}", f"waypoint_{response['waypoint_name']}_Scene.png")
                print("[Parent] Press y to confirm human is present, or any other key to continue searching. Automatically closes in 10 seconds.")

                img = cv2.imread(image_path)
                cv2.imshow("Potential Target", img) ## comment these back in when testing it will show u what it is seeing
                key_press = cv2.waitKey(10000) # wait for 10 seconds for user input 
                cv2.destroyAllWindows()
                cv2.waitKey(1) # needed to ensure cv2 closes properly




                # if the user confirms the human is present, set the target_found variable to True
                if key_press == ord('y'):
                    print(f"[Parent] User confirmed human is present at world coordinates: {all_waypoints_dictionary[response['waypoint_name']][1]}")
                    print(f"[Parent] GPS coordinates: {all_waypoints_dictionary[response['waypoint_name']][0]}")
                    target_found.value = True
                    SHUTDOWN_EVENT.set() # set the shutdown event to stop the simulation

                    # change the image name to indicate it was the intended target
                    new_image_path = os.path.join(img_dir, f"drone_{drone_name}", f"waypoint_{response['waypoint_name']}_Scene_CORRECT_TARGET.png")
                    os.rename(image_path, new_image_path)        
                    #print(f"Renamed image to {new_image_path}")             
                else:
                    print("[Parent] User did not confirm human is present. Continuing search...")

                    # change the image name to indicate it was a potential target
                    new_image_path = os.path.join(img_dir, f"drone_{drone_name}", f"waypoint_{response['waypoint_name']}_Scene_POTENTIAL_TARGET.png")
                    os.rename(image_path, new_image_path)
                    #print(f"Renamed image to {new_image_path}")
                
                print(f"[Drone {drone_name}] Continuing to search.")
            else:
                print(f"[Parent] Drone {drone_name} did not find a human in the image taken at waypoint {response['waypoint_name']}")

            # remove the request from the dictionary
            VLM_RESPONSE_DICTIONARY[drone_name] = None
    
    #print("[Parent] VLM Response Processed")



def loop():
    '''Perform loop until end of simulation'''

    #print("[Parent] Entering Loop")

    while not SHUTDOWN_EVENT.is_set() and not target_found.value:
        #print("Loop running...")
        websocket_data = fetch_websocket_data()
        process_websocket_message(websocket_data)

        assign_waypoints()
        send_status_update()
        send_target_update()
        delete_searched_waypoints()
        process_image_queue()
        process_vlm_response()

        

        
        #print(f"[Parent] Waypoint Queue: {[wp.name for wp in waypoint_queue]}")
        #print(f"Received UI queue: {[item for item in RECEIVED_UI_DATA_QUEUE.queue]}")
        time.sleep(1)


    #print("[Parent] Exiting Loop")

    

def start_parent_controller(drone_count: int, shutdown_event):
    
    try:
        global img_dir
        img_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'images')

        initialize(drone_count, shutdown_event)
        perform_startup_sequence()
        receive_initial_waypoints()

        for id in range(DRONE_COUNT):
            initialize_drone_controller(id)

        while not all(status == "IDLE" for status in status_dictionary.values()):
            print("Waiting for all drones to take off...")
            time.sleep(5)

        # send to ui that all drones are ready
        send_status_update()

        loop()

    except KeyboardInterrupt:
        SHUTDOWN_EVENT.set()

    finally:
        for process in drone_controller_processes:
            if process.is_alive():
                process.terminate()
                process.join()
        if websocket_client_thread and websocket_client_thread.is_alive():
            websocket_client_thread.join()
        if VLM_thread and VLM_thread.is_alive():
            VLM_thread.join()
        print("Parent Controller terminated successfully.")
