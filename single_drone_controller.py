import airsim  # type: ignore
import multiprocessing as mp
import random
import time
import os
import numpy as np
import cv2
import local_config
import base64
import heapq
import asyncio


from classes import Image
from helper_functions import unreal_to_gps
import search_functions

MIN_ALTITUDE = 10
VELOCITY = 15

WAYPOINT_ALTITUDE = -15            # Fixed altitude (negative for AirSim)
WAYPOINT_SIDE_LENGTH = 10          # Square size
WAYPOINT_SPEED = 8                 # Speed (m/s)




# Enables api control, takes off drone, returns the client
def takeOff(drone_name):
    client = airsim.MultirotorClient(local_config.LOCAL_IP)
    client.confirmConnection()
    client.enableApiControl(True, drone_name)
    client.armDisarm(True, drone_name)
    client.takeoffAsync(vehicle_name=drone_name).join()

    # print("Drone " + drone_name + " is ready to fly")

    return client







def singleDroneController(drone_name, current_target_dictionary, status_dictionary, target_found, searched_areas_dictionary, image_queue, waypoint_queue):
    """ Drone process that listens for movement commands and sends status updates. """
    
    # Initialize AirSim client and take off
    client = takeOff(drone_name)


    def take_forward_picture(drone_name, image_type):
        camera_name = "front-" + drone_name
        print(f"Taking picture from {camera_name}, type of {image_type}")
        response = client.simGetImage(camera_name=camera_name, image_type=image_type, vehicle_name=drone_name)
        
        filename = os.path.join("images", f"{camera_name}_scene_{image_type}")

        img1d = np.frombuffer(response, dtype=np.uint8)


        if image_type == airsim.ImageType.Infrared:
            img_gray = cv2.imdecode(img1d, cv2.IMREAD_GRAYSCALE)
            cv2.imwrite(os.path.normpath(filename + '.png'), img_gray) # write to png on disk
            _, buffer = cv2.imencode('.png', img_gray)  # Encode the image as PNG
            base64_image = base64.b64encode(buffer).decode('utf-8')  # Convert to base64
        else:
            img_rgb = cv2.imdecode(img1d, cv2.IMREAD_COLOR)
            cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png on disk
            _, buffer = cv2.imencode('.png', img_rgb)  # Encode the image as PNG
            base64_image = base64.b64encode(buffer).decode('utf-8')  # Convert to base64

    
        return base64_image  



    while not target_found.value:
        current_target = current_target_dictionary[drone_name]

        if current_target is not None:
            waypoint_name = current_target.name
            waypoint_lat, waypoint_lon, waypoint_alt = unreal_to_gps(current_target.x, current_target.y, current_target.z, client.getHomeGeoPoint())
            waypoint_x = current_target.x
            waypoint_y = current_target.y
            waypoint_z = current_target.z

            print(f"Drone {drone_name} is moving to {waypoint_name}")
            print("Going to GPS coordinates: ", waypoint_lat, waypoint_lon, waypoint_alt)
            print("Going to Unreal coordinates: ", waypoint_x, waypoint_y, waypoint_z)


            move_future = client.moveToGPSAsync(waypoint_lat, waypoint_lon, waypoint_alt + MIN_ALTITUDE, VELOCITY, vehicle_name=drone_name)
            status_dictionary[drone_name] = "MOVING"

            while True:
                # if current_target_dictionary[drone_name] != original_target: (TODO: Fix and test this)
                #     print(f"Drone {drone_name} received a new target while moving to {waypoint_name}")
                #     # interrupt movement with hover
                #     client.hoverAsync().join()

                #     # push the original target back to the queue to be revisited later
                #     heapq.heappush(waypoint_queue, original_target)
                #     break

                drone_state = client.getMultirotorState(vehicle_name=drone_name)
                position = drone_state.kinematics_estimated.position
                current_x, current_y, current_z = position.x_val, position.y_val, position.z_val


                print("Recorded height: ", client.getDistanceSensorData(distance_sensor_name='Distance', vehicle_name=drone_name).distance)
                if (client.getDistanceSensorData(distance_sensor_name='Distance', vehicle_name=drone_name).distance < MIN_ALTITUDE or client.getDistanceSensorData(distance_sensor_name='Distance2', vehicle_name=drone_name).distance < MIN_ALTITUDE):
                    gpsData = drone_state.gps_location
                    #client.moveToGPSAsync(gpsData.latitude, gpsData.longitude, gpsData.altitude+5, VELOCITY / 2, vehicle_name=drone_name).join()
                    # move z up
                    client.hoverAsync().join()
                    print("current height was: " + str(current_z))
                    client.moveToZAsync(current_z - (MIN_ALTITUDE / 2), VELOCITY / 2, vehicle_name=drone_name).join()
                    print("moving up to ", current_z - (MIN_ALTITUDE / 2))
                    move_future = client.moveToGPSAsync(waypoint_lat, waypoint_lon, waypoint_alt + MIN_ALTITUDE, VELOCITY, vehicle_name=drone_name)

                    print("continuing to: " + str(waypoint_lat) + " " + str(waypoint_lon) + " " + str(waypoint_alt - MIN_ALTITUDE))


                # Check if the drone gps is close enough to the target (within a small threshold)
                distance = ((current_x - waypoint_x)**2 + (current_y - waypoint_y)**2)**0.5
                #print("Distance to target: ", distance)
                if distance < 5.0:  # 5-meter tolerance
                    move_future.join()
                    print(f"Drone {drone_name} reached the target")
                    break  

                time.sleep(1)
            status_dictionary[drone_name] = "SEARCHING"
            print(f"Drone {drone_name} arrived and is searching {waypoint_name}")

            # hover for 5 seconds
            client.hoverAsync(vehicle_name=drone_name).join()
            # go down 20 meters
            client.moveToZAsync(-20, WAYPOINT_SPEED, vehicle_name=drone_name).join()
            # rotate left 30 degrees
            client.rotateToYawAsync(-30, vehicle_name=drone_name).join()
            time.sleep(5)

            print("Calling search function")
            drone_state = client.getMultirotorState(vehicle_name=drone_name)
            position = drone_state.kinematics_estimated.position
            current_x, current_y, current_z = position.x_val, position.y_val, position.z_val
            print("Current position: ", current_x, current_y, current_z)

            search_functions.waypoint_search(client, drone_name, current_x, current_y, WAYPOINT_SIDE_LENGTH, current_z, WAYPOINT_SPEED)
            print("Search function finished")
            # Take a picture
            # base64_picture = take_forward_picture(drone_name, airsim.ImageType.Scene)

            # image_queue.put(Image(drone_name, "Scene", base64_picture, current_target.name))



            # time.sleep(5)

            current_target_dictionary[drone_name] = None
            print(f"Drone {drone_name} finished searching {waypoint_name}")
            searched_areas_dictionary[waypoint_name] = (waypoint_lat, waypoint_lon, waypoint_alt)
            status_dictionary[drone_name] = "IDLE"


        else:
            print(f"Drone {drone_name} is waiting for commands.")
            status_dictionary[drone_name] = "IDLE"
            time.sleep(10)




if __name__ == "__main__":
    print("This script is not meant to be run directly. Please run mission_control.py instead.")
    exit(1)