import airsim # type: ignore
import os
import numpy as np
import cv2
import pprint
import time


# ===================================== SETUP =====================================

# set up client object to access multirotor drones
client = airsim.MultirotorClient()

# connect to AirSim simulator
client.confirmConnection()

# directory to store pictures
imgDir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'images')

# create directory if it does not exist
if not os.path.exists(imgDir):
    os.makedirs(imgDir)

# =================================================================================






# moves the drone relative to its current position
def move_drone_relative(drone_name, x, y, z, speed):
    state = client.getMultirotorState(vehicle_name=drone_name)
    current_position = state.kinematics_estimated.position
    #print("Current position: " + str(current_position))
    new_position = airsim.Vector3r(current_position.x_val + x, current_position.y_val + y, current_position.z_val + z)    
    return client.moveToPositionAsync(new_position.x_val, new_position.y_val, new_position.z_val, speed, vehicle_name=drone_name)



# moves the drone to a specific GPS coordinate
def move_drone_GPS(drone_name, lat, lon, alt, speed):
    print("Original position: " + str(get_GPS_coordinates(drone_name)))
    print("Moving to GPS coordinates: " + str(lat) + ", " + str(lon) + ", " + str(alt))
    return client.moveToGPSAsync(lat, lon, alt, speed, vehicle_name=drone_name)





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


# prints the GPS coordinates of the drone
def print_GPS_coordinates(drone_name):
    print("===============================================================")
    coords = get_GPS_coordinates(drone_name)
    print("GPS coordinates: " + str(coords))



drone = "0"



client.enableApiControl(True, vehicle_name=drone)
client.armDisarm(True, vehicle_name=drone)


client.takeoffAsync(vehicle_name=drone).join()


# move drone up 25 meters
move_drone_relative(drone, 0, 0, -25, 5).join()

# get initial coordinates
initial_coords = get_GPS_coordinates(drone)
print("Initial coordinates: " + str(initial_coords))

# move drone left 20 meters
f2 = move_drone_relative(drone, 0, -20, 0, 5).join()


# get final coordinates
final_coords = get_GPS_coordinates(drone)
print("Final coordinates: " + str(final_coords))


# calcualte the distance moved
distance_moved = np.sqrt((final_coords.latitude - initial_coords.latitude)**2 + (final_coords.longitude - initial_coords.longitude)**2)


print("Distance moved: " + str(distance_moved))



# end connection
client.enableApiControl(False, vehicle_name=drone)
