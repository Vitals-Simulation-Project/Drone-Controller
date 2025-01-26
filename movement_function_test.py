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



# prints the state of the drone
def print_state(drone_name):
    print("===============================================================")
    state = client.getMultirotorState(vehicle_name=drone_name)
    print("state: %s" % pprint.pformat(state))
    return state


# gets GPS coordinates of the drone
def get_GPS_coordinates(drone_name):
    state = client.getMultirotorState(vehicle_name=drone_name)
    return state.gps_location


drone = "0"
# drone2 = "1"

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


f1 = client.takeoffAsync(vehicle_name=drone).join()




# move drone up 25 meters
f1 = move_drone(drone, 0, 0, -25, 5).join()

# get initial coordinates
initial_coords = get_GPS_coordinates(drone)
print("Initial coordinates: " + str(initial_coords))

f1 = move_drone(drone, 0, -20, 0, 5).join()

print("Moving back to initial position")
client.moveToGPSAsync(initial_coords.latitude, initial_coords.longitude, initial_coords.altitude, 5, vehicle_name=drone).join()

# get final coordinates
final_coords = get_GPS_coordinates(drone)
print("Final coordinates: " + str(final_coords))


# calcualte the distance moved
distance_moved = np.sqrt((final_coords.latitude - initial_coords.latitude)**2 + (final_coords.longitude - initial_coords.longitude)**2)


print("Distance moved: " + str(distance_moved))



# end connection
client.enableApiControl(False, vehicle_name=drone)
