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


# Define the Message class for the model to use in its responses
class Message(BaseModel):
    drone_id: int
    done: bool
    tool_calls: list



# the tools that the model can call
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



# prints the state of the drone
def print_state():
    print("===============================================================")
    state = client.getMultirotorState()
    print("state: %s" % pprint.pformat(state))
    return state





MODEL = "llama3.2:latest" # model from Ollama
URL = "http://localhost:11434/api/chat" 





# pull the model
if pull_model(MODEL) == "success":
    print("Model " + MODEL + " successfully pulled.")
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

message_history = [
        {
            "role": "user",
            "content": "CONTEXT: You control 5 drones by first sending a single number in the range of 0 to 4 specifying which drone to control. \
            then you use a function call to decide what to do with the drone. \
            The function calls are: takeoff, land, move, and take pictures. \
            Before you can use the function calls, you must first takeoff the drone. \
            You can only control one drone at a time. Use a function call to move any one drone to a position of your choice. \
            Then when you are done, land the drone and set the bool 'done' to true."        
        }
    ],

response = chat(
    messages = message_history,
    model = MODEL,
    format = Message.model_json_schema(),
    tools = [takeoff, land, move, take_pictures]
)



message = Message.model_validate_json(response.message.content)
print(str(message))


while True:
    if message.done:
        break

    message = Message.model_validate_json(response.message.content)

    print("Model response: " + response.message.content)
    print("Drone ID: " + str(message.drone_id))

    if response.message.tool_calls:
        print("A tool call was made.")

        for tool in response.message.tool_calls or []:
            print("Checking tool: " + tool.function.name)
            if function_to_call := available_tools.get(tool.function.name):
                print("Calling function: " + tool.function.name)
                print("Arguments: " + str(tool.function.arguments))
                output = function_to_call(**tool.function.arguments)
                print("Function output: " + str(output))
            else:
                print("Function " + tool.function.name + " not found")


    # Only needed to chat with the model using the tool call results
    if response.message.tool_calls:
        # Add the function response to messages for the model to use
        message_history.append(response.message)
        message_history.append({'role': 'tool', 'content': str(output), 'name': tool.function.name})

        # Get final response from model with function outputs
        final_response = chat('llama3.2:latest', messages=message_history)
        print('Final response:', final_response.message.content)
    
    response = chat(
        messages = [
            {
                "role": "user",
                "content": "CONTEXT: You control 5 drones by first sending a single number in the range of 0 to 4 specifying which drone to control. \
                then you use a function call to decide what to do with the drone. \
                The function calls are: takeoff, land, move, and take pictures. \
                Before you can use the function calls, you must first takeoff the drone. \
                You can only control one drone at a time. Use a function call to move any one drone to a position of your choice. \
                Then when you are done, land the drone and set the bool 'done' to true."        
            }
        ],
        model = MODEL,
        format = Message.model_json_schema(),
        tools = [takeoff, land, move, take_pictures]
    )





