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





def takeoff(drone: int):
    print("Taking off drone " + str(drone))
    f1 = client.takeoffAsync(vehicle_name=str(drone))
    f1.join()

def land(drone: int):
    print("Landing drone " + str(drone))
    f1 = client.landAsync(vehicle_name=str(drone))
    f1.join()

def move(drone: int, x: float, y: float, z: float):
    print("Moving drone " + str(drone) + " to position: " + str(x) + ", " + str(y) + ", " + str(z))
    f1 = client.moveToPositionAsync(x, y, z, 3, vehicle_name=str(drone))
    f1.join()

def take_pictures(drone: int):
    print("Taking pictures")
    return
    f1 = client.simSetCameraOrientation("0", airsim.to_quaternion(0, 0, 0), vehicle_name=str(drone))
    f1 = client.simSetCameraOrientation("0", airsim.to_quaternion(0, 0, 0), vehicle_name=str(drone))
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
    ], vehicle_name=str(drone))
    for i, response in enumerate(responses):
        filename = os.path.normpath('C:/temp/py' + str(i))
        if not os.path.exists(os.path.dirname(filename)):
            os.makedirs(os.path.dirname(filename))
        if response.pixels_as_float:
            airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
        else:
            airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
    

available_tools = {
    "takeoff": takeoff,
    "land": land,
    "move": move,
    "take_pictures": take_pictures
}


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
            "content": "CONTEXT: You control 5 drones by first sending a single number in the range of 0 to 4 specifying which drone to control. \
            then you use a function call to decide what to do with the drone. \
            The function calls are: takeoff, land, move, and take pictures. \
            Before you can use the function calls, you must first takeoff the drone. \
            You can only control one drone at a time. "            
        }
    ],
    model = MODEL,
    format = Message.model_json_schema(),
    tools = [takeoff, land, move, take_pictures]
)

message = Message.model_validate_json(response.message.content)
print(message)

for tool in response.message.tool_calls or []:
    function_to_call = available_tools[tool]
    if function_to_call:
        function_to_call(**tool.function.arguments)
    else:
        print("Tool not found: " + tool.function.name)



