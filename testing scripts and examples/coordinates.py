import airsim # type: ignore
import os
import numpy as np
import cv2
import pprint
import time



def take_forward_picture(drone_name, image_type):
    camera_name = "front-" + drone_name
    print(f"Taking picture from {camera_name}")
    response = client.simGetImage(camera_name=camera_name, image_type=image_type, vehicle_name=drone_name)
    
    filename = os.path.join("images", f"{camera_name}_scene_{image_type}")

    img1d = np.frombuffer(response, dtype=np.uint8)
    img_rgb = cv2.imdecode(img1d, cv2.IMREAD_COLOR)

    cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png

def move_drone_relative(drone_name, x, y, z, speed):
    state = client.getMultirotorState(vehicle_name=drone_name)
    current_position = state.kinematics_estimated.position
    new_position = airsim.Vector3r(current_position.x_val + x, current_position.y_val + y, current_position.z_val + z)    
    client.moveToPositionAsync(new_position.x_val, new_position.y_val, new_position.z_val, speed, vehicle_name=drone_name).join()
    return new_position

drone = "0"

# directory to store pictures
imgDir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'images')

# create directory if it does not exist
if not os.path.exists(imgDir):
    os.makedirs(imgDir)

# set up client object to access multirotor drone
client = airsim.MultirotorClient()

# connect to AirSim simulator
client.confirmConnection()
client.enableApiControl(True, vehicle_name=drone)
client.armDisarm(True, vehicle_name=drone)

client.takeoffAsync(vehicle_name=drone).join()

client.moveToPositionAsync(220, -10, -50, 10, vehicle_name=drone).join()

# print current coordinates
state = client.getMultirotorState(vehicle_name=drone)
current_position = state.kinematics_estimated.position
print(f"Current position: {current_position.x_val}, {current_position.y_val}, {current_position.z_val}")

# print gps coordinates
print(f"GPS coordinates: {state.gps_location.latitude}, {state.gps_location.longitude}, {state.gps_location.altitude}")




# end connection
client.enableApiControl(False)
