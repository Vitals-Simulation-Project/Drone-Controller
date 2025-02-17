import airsim  # type: ignore
import multiprocessing as mp
import random
import time
import os
import numpy as np
import cv2
import requests
import json
from ollama import chat # type: ignore
from pydantic import BaseModel

# project imports
import local_config
import single_drone_controller as sdc
from model_files.pull_model import load_model


# poi1 is a small shack in front of the spawn area
POINTS_OF_INTEREST = [
    {"name": "POI1", "location": [0.0020828851586628867, -5.421523784302704e-05, 49.92039108276367]},
]


MODEL = "llava:7b" # model from Ollama
URL = "http://localhost:11434/api/chat" 

class Message(BaseModel):
    truth_value: bool
    topic: str

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



# make a global queue for images to process




# send one message to load model
response = chat(
    'llama3.2',
    messages=[
    {'role': 'user', 'content': "hello"},
    ],
    stream=False,
    format = Message.model_json_schema()
)



def parentController(drone_count):
    """ Parent process to send commands and receive status updates from drones. """
    mp.set_start_method('spawn')  # Windows-specific start method

    # send one message to load model
    response = chat(
        'llama3.2',
        messages=[
        {'role': 'user', 'content': "hello"},
        ],
        stream=False,
    )
    print(response.message.content)

    input("Press enter to continue")
    
    command_queues = {}  # Dictionary to store queues for sending commands
    status_queue = mp.Queue()  # Single queue for receiving updates
    processes = []  # List to store process references

    # Create and start processes
    for x in range(drone_count):
        drone_name = str(x)
        command_queues[drone_name] = mp.Queue()
        p = mp.Process(target=sdc.singleDroneController, args=(drone_name, drone_count, command_queues[drone_name], status_queue))
        p.start()
        processes.append(p)


    # Wait for all drones to be ready
    while status_queue.qsize() < drone_count:
        print("Waiting for drones to be ready...")
        time.sleep(5)
    
    # pop all messages from the queue and print drone status
    while not status_queue.empty():
        drone_name, status = status_queue.get()
        print(f"Drone {drone_name}: {status}")



    try:
        while True:
            # Receive status updates from drones
            while not status_queue.empty():
                drone_name, status = status_queue.get()
                print(f"Update from Drone {drone_name}: {status}")

            user_input = input("Enter 'rand' for random movement or 'STOP' to quit: ")

            if user_input == "STOP":
                for q in command_queues.values():
                    q.put("STOP")  # Stop all drones
                break

            elif user_input == "rand":
                for drone_name, q in command_queues.items():
                    x = random.uniform(-50, 50)
                    y = random.uniform(-50, 50)
                    z = random.uniform(-20, -5)  # Stay within flight limits
                    q.put((x, y, z))  # Send position command to drone
            else:
                # Parse the input
                try:
                    x, y, z = map(float, user_input.split())
                    for q in command_queues.values():
                        q.put((x, y, z))
                except ValueError:
                    print("Invalid input. Type 'rand' for random movement or 'STOP' to exit.")

            time.sleep(1)  # Give some time for status updates
    
    except KeyboardInterrupt:
        for q in command_queues.values():
            q.put("STOP")

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
