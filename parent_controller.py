#=========================
#  parent_controller.py  #
#=========================

# Parent Process that manages communication between VLM, UI, and Drones

# Python Dependencies
import asyncio
import heapq
import json
import multiprocessing as mp
from ollama import chat
import os
from queue import Queue
import requests
import threading
import time
import websockets

# Project Imports
from classes import Image, VLMOutput, Waypoint
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

VLM_RESPONSE_LIST = {} # dictionary to store VLM responses, maps request number to the response
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
target_found = None                 # global variable to stop the search loop when the target is found
searched_areas_dictionary = None    # dictionary mapping waypoint names to their locations that have been searched already
image_queue = None                  # queue to store images waiting to be processed
waypoint_queue = None               # priority queue to store waypoints

# Keep processes and threads global to avoid early shutdown
drone_controller_processes = []
websocket_client_thread = None
VLM_thread = None

# Shutdown Event
SHUTDOWN_EVENT = None

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
    current_target_dictionary[drone_name] = None
    status_dictionary[drone_name] = None
    request_dictionary[drone_name] = None
    current_position_dictionary[drone_name] = None

    drone_state_message = {
        "MessageType": "UpdateDroneState",
        "DroneID": id,
        "State": "INITIALIZING"
    }

    send_to_ui(json.dumps(drone_state_message))

    process = mp.Process(target=sdc.singleDroneController, args=(drone_name, current_target_dictionary, status_dictionary, target_found, searched_areas_dictionary, image_queue, waypoint_queue, current_position_dictionary))
    process.start()
    drone_controller_processes.append(process)

    print(f"[Drone {drone_name}] Initializing")

def loop():
    '''Perform loop until end of simulation'''

    print("[Parent] Entering Loop")

    while not SHUTDOWN_EVENT.is_set() and not target_found.value:
        
        websocket_data = fetch_websocket_data()
        process_websocket_message(websocket_data)

        assign_waypoints()
        send_status_update()
        send_target_update()
        delete_searched_waypoints()

        time.sleep(5)


    print("[Parent] Exiting Loop")


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

    for drone_name in status_dictionary:

        if status_dictionary[drone_name] == "WAITING" or status_dictionary[drone_name] == "IDLE":
            if len(waypoint_queue) > 0:
                next_waypoint = heapq.heappop(waypoint_queue)
                current_target_dictionary[drone_name] = next_waypoint
                status_dictionary[drone_name] = "MOVING"
                print(f"[Drone {drone_name}] Assigned Waypoint: {next_waypoint.name}")

def delete_searched_waypoints():

    for waypoint in searched_areas_dictionary:
        delete_message = {
            "MessageType": "DeleteWaypoint",
            "ID": waypoint
        }

        send_to_ui(json.dumps(delete_message))



def send_request_to_VLM(request):
    """Send a request to the VLM model and return the response in the queue"""

    global request_number
    request_id = request_number # store the request number for the response
    request_number += 1 # increment the request number for the next request

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

    if websocket_data:
        
        json_data = json.loads(websocket_data)

        if json_data["MessageType"] == "AddWaypoint":

            # divide by 100 to convert from cm to m
            waypoint = Waypoint(json_data["WaypointID"], json_data["X"] / 100, json_data["Y"] / 100, json_data["Z"] / 100, json_data["Priority"])
            heapq.heappush(waypoint_queue, waypoint)
            print(f"\n[OVERRIDE] Added waypoint {waypoint.name} to the queue\n")

        elif json_data["MessageType"] == "DeleteWaypoint":

            # delete the waypoint from the queue
            for i, wp in enumerate(waypoint_queue):
                if wp.name == json_data["WaypointID"]:
                    del waypoint_queue[i]
                    print(f"\n[OVERRIDE] Deleted waypoint {wp.name} from the queue\n")
                    break

            for drone_name in current_target_dictionary:
                if current_target_dictionary[drone_name] == json_data["WaypointID"]:
                    current_target_dictionary[drone_name] = None
                    status_dictionary[drone_name] = "WAITING"

            # fix the heap
            heapq.heapify(waypoint_queue)

        elif json_data["MessageType"] == "StopSimulation":
            print("Stopping simulation...")
            SHUTDOWN_EVENT.set()               
        else:
            print("Unexpected message type")



def start_parent_controller(drone_count: int, shutdown_event):

    try:
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
        ...
