import airsim # type: ignore
import os
import numpy as np
import cv2
import pprint
import time



# a function that moves the drone relative to its current position
def move_drone(drone_name, x, y, z, speed):
    state = client.getMultirotorState(vehicle_name=drone_name)
    current_position = state.kinematics_estimated.position
    new_position = airsim.Vector3r(current_position.x_val + x, current_position.y_val + y, current_position.z_val + z)    
    return client.moveToPositionAsync(new_position.x_val, new_position.y_val, new_position.z_val, speed, vehicle_name=drone_name)


drone = "0"
drone2 = "1"

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

client.enableApiControl(True, vehicle_name=drone2)
client.armDisarm(True, vehicle_name=drone2)


f1 = client.takeoffAsync(vehicle_name=drone)
f2 = client.takeoffAsync(vehicle_name=drone2)
f1.join()
f2.join()

# move drone up 25 meters
f1 = move_drone(drone, 0, 0, -25, 5)
f2 = move_drone(drone2, 0, 0, -25, 5)

f1.join()
f2.join()

f1 = move_drone(drone, 0, 25, 0, 5)
f2 = move_drone(drone2, 0, 25, 0, 5)

f1.join()
f2.join()



# end connection
client.enableApiControl(False, vehicle_name=drone)
client.enableApiControl(False, vehicle_name=drone2)
