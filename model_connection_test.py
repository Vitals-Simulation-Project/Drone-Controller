import airsim # type: ignore
import requests
import json
import time
import os
import statistics
import pprint

from model_files.pull_model import pull_model

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

data = {
    "model": MODEL,
    "messages": [
        {
            "role": "user",
            "content": "Create a course in the shape of a circle using the xyz coordinate system. The circle should have a radius of 5 and be centered at the (0, 0, 0). Return the answer as 4 lines of 3 numbers separated by spaces. Each line in the output represents a point of the circle, for example the northenmost point of the circle would be 0 5 0. Only output the coordinates of the circle, do not include any other information."
        }
    ],
    "stream": False
}


response = requests.post(URL, json=data)
response_json = json.loads(response.text)


print(response_json["message"]["content"])

lines = response_json["message"]["content"].split("\n")

# get the xyz coordinates from the response
coordinates_list = []
for line in lines:
    coordinates = line.split(" ")
    coordinates_list.append([int(x) for x in coordinates])

# move the drone to the coordinates
client.moveToPositionAsync(coordinates_list[0][0], coordinates_list[0][1], coordinates_list[0][2], 5, vehicle_name="0").join()
client.moveToPositionAsync(coordinates_list[1][0], coordinates_list[1][1], coordinates_list[1][2], 5, vehicle_name="0").join()
client.moveToPositionAsync(coordinates_list[2][0], coordinates_list[2][1], coordinates_list[2][2], 5, vehicle_name="0").join()




