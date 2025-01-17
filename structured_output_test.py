import airsim # type: ignore
import requests
import json
import time
import os
import statistics
import pprint

from ollama import chat # type: ignore
from pydantic import BaseModel

from model_files.pull_model import pull_model


class Message(BaseModel):
    coordinates: list[list[float]]
    



def print_state():
    print("===============================================================")
    state = client.getMultirotorState()
    print("state: %s" % pprint.pformat(state))
    return state

MODEL = "llava:7b" # model from Ollama
URL = "http://localhost:11434/api/chat" 


# pull the model
if pull_model(MODEL) == "success":
    print("Model successfully pulled.")
else:
    print("Model pull failed.")
    exit()

# send one request to get the model loaded
data = {
    "model": MODEL,
    "messages": [
        {
            "role": "user",
            "content": "Hello"
        }
    ],
    "stream": False
}

response = requests.post(URL, json=data)
response_json = json.loads(response.text)
print(response_json["message"]["content"])


# wait a few seconds to make sure the model is loaded
time.sleep(5)
input("Ensure all other software is closed. Press Enter to start the test.")



# set up client object to access multirotor drone
client = airsim.MultirotorClient()

# connect to AirSim simulator
client.confirmConnection()
client.enableApiControl(True, vehicle_name="0")
client.armDisarm(True, vehicle_name="0")

client.getMultirotorState(vehicle_name="0")
state = print_state()



response = chat(
    messages = [
        {
            "role": "user",
            "content": "CONTEXT: You control a single drone by sending it a series of coordinates. \
            The drone will fly to each coordinate in order. The coordinates are in the form of a list of lists of 3 float numbers. \
            For example, [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]] would be two coordinates. The coordinates in each sub-list are x position, y position, and z position, \
            respectively. COMMAND: Move the drone in a square pattern by creating a list of coordinates centered around 0 0 0."
        }
    ],
    model = MODEL,
    format = Message.model_json_schema()
)

message = Message.model_validate_json(response.message.content)
print(message)

for i in range(len(message.coordinates)):
    client.moveToPositionAsync(message.coordinates[i][0], message.coordinates[i][1], message.coordinates[i][2], 5, vehicle_name="0").join()
