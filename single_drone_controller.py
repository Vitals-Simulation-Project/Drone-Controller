import airsim  # type: ignore
import multiprocessing as mp
import random
import time
import os
import numpy as np
import cv2
import local_config
import geopy.distance # type: ignore

# Enables api control, takes off drone, returns the client
def takeOff(droneName):
    client = airsim.MultirotorClient(local_config.LOCAL_IP)
    client.confirmConnection()
    client.enableApiControl(True, droneName)
    client.armDisarm(True, droneName)
    client.takeoffAsync(vehicle_name=droneName).join()

    # print("Drone " + droneName + " is ready to fly")

    return client



# Convert Unreal Engine coordinates to GPS
def unreal_to_gps(ue_x, ue_y, ue_z, home_gps):
    """
    Converts Unreal Engine (UE) coordinates to GPS coordinates.
    """
    home_lat, home_lon, home_alt = home_gps.latitude, home_gps.longitude, home_gps.altitude

    # Convert UE X and Y to GPS using geopy
    new_lat_lon = geopy.distance.distance(meters=ue_x).destination((home_lat, home_lon), bearing=0)  # North-South
    # Use the resulting tuple and then apply the Y conversion (East-West)
    new_lat_lon = geopy.distance.distance(meters=ue_y).destination(new_lat_lon, bearing=90)  # East-West

    new_lat = new_lat_lon[0]  # Extract latitude
    new_lon = new_lat_lon[1]  # Extract longitude

    # Convert UE Z to GPS Altitude (UE Z is negative when going up)
    new_alt = home_alt - ue_z  

    return new_lat, new_lon, new_alt



def singleDroneController(droneName, current_target_dictionary, status_dictionary, target_found, searched_areas):
    """ Drone process that listens for movement commands and sends status updates. """
    #print(f"In single controller, Drone name: {droneName}")

    
    
    # Initialize AirSim client and take off
    client = takeOff(droneName)


    def move_drone_relative(drone_name, x, y, z, speed):
        state = client.getMultirotorState(vehicle_name=drone_name)
        current_position = state.kinematics_estimated.position
        new_position = airsim.Vector3r(current_position.x_val + x, current_position.y_val + y, current_position.z_val + z)    
        client.moveToPositionAsync(new_position.x_val, new_position.y_val, new_position.z_val, speed, vehicle_name=drone_name).join()
        return new_position


    
    def take_forward_picture(drone_name, image_type):
        camera_name = "front-" + drone_name
        print(f"Taking picture from {camera_name}")
        response = client.simGetImage(camera_name=camera_name, image_type=image_type, vehicle_name=drone_name)
        
        filename = os.path.join("images", f"{camera_name}_scene_{image_type}")

        img1d = np.frombuffer(response, dtype=np.uint8)
        img_rgb = cv2.imdecode(img1d, cv2.IMREAD_COLOR)

        cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png


    while not target_found.value:
        current_target = current_target_dictionary[droneName]

        if current_target is not None:
            waypoint_name = current_target.name
            waypoint_lat, waypoint_lon, waypoint_alt = unreal_to_gps(current_target.x, current_target.y, current_target.z, client.getHomeGeoPoint())
            waypoint_x = current_target.x
            waypoint_y = current_target.y
            waypoint_z = current_target.z

            print("Going to GPS coordinates: ", waypoint_lat, waypoint_lon, waypoint_alt)

            # print(f"Received command: Drone {droneName} is moving to {waypoint_name} at {waypoint_x}, {waypoint_y}, {waypoint_z}")
            # print(f"Current position: {client.getMultirotorState(vehicle_name=droneName).kinematics_estimated.position}")
            #move_future = client.moveToPositionAsync(waypoint_x, waypoint_y, waypoint_z, 5, vehicle_name=droneName)
            move_future = client.moveToGPSAsync(waypoint_lat, waypoint_lon, waypoint_alt, 15, vehicle_name=droneName)
            status_dictionary[droneName] = "MOVING"

            current_time = time.time()
            while True:
                drone_state = client.getMultirotorState(vehicle_name=droneName)
                position = drone_state.kinematics_estimated.position
                current_x, current_y, current_z = position.x_val, position.y_val, position.z_val


                # Check if the drone is close enough to the target (within a small threshold)
                distance = ((current_x - waypoint_x) ** 2 + (current_y - waypoint_y) ** 2 + (current_z - waypoint_z) ** 2) ** 0.5
                if distance < 1.0:  # 1-meter tolerance
                    move_future.join()
                    print(f"Drone {droneName} reached the target")
                    break  

                time.sleep(1)
                print("Drone is moving")
            status_dictionary[droneName] = "SEARCHING"
            print(f"Drone {droneName} arrived and is searching {waypoint_name}")

            time.sleep(60)

            current_target_dictionary[droneName] = None
            print(f"Drone {droneName} finished searching {waypoint_name}")
            searched_areas[waypoint_name] = (waypoint_lat, waypoint_lon, waypoint_alt)
            status_dictionary[droneName] = "WAITING"


        else:
            print(f"Drone {droneName} is waiting for commands.")
            status_dictionary[droneName] = "WAITING"
            time.sleep(5)




if __name__ == "__main__":
    print("This script is not meant to be run directly.")