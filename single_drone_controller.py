import airsim  # type: ignore
import multiprocessing as mp
import random
import time
import os
import numpy as np
import cv2
import local_config

# Enables api control, takes off drone, returns the client
def takeOff(droneName):
    client = airsim.MultirotorClient(local_config.LOCAL_IP)
    client.confirmConnection()
    client.enableApiControl(True, droneName)
    client.armDisarm(True, droneName)
    client.takeoffAsync(vehicle_name=droneName).join()

    # print("Drone " + droneName + " is ready to fly")

    return client

def singleDroneController(droneName, droneCount, current_target, status, TARGET_FOUND):
    """ Drone process that listens for movement commands and sends status updates. """
    #print(f"In single controller, Drone name: {droneName}")
    
    # Initialize AirSim client and take off
    client = takeOff(droneName)
    status = "READY"

    def move_drone_relative(drone_name, x, y, z, speed):
        state = client.getMultirotorState(vehicle_name=drone_name)
        current_position = state.kinematics_estimated.position
        new_position = airsim.Vector3r(current_position.x_val + x, current_position.y_val + y, current_position.z_val + z)    
        client.moveToPositionAsync(new_position.x_val, new_position.y_val, new_position.z_val, speed, vehicle_name=drone_name).join()
        return new_position

    def move_drone_absolute(drone_name, x, y, z, speed):
        new_position = airsim.Vector3r(x, y, z)
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


    while not TARGET_FOUND:

        if current_target is not None:
            waypoint_name, waypoint_location = current_target
            print(f"Received command: Drone {droneName} is moving to {waypoint_name} at {waypoint_location}")
            new = client.moveToPositionAsync(waypoint_location.x_val, waypoint_location.y_val, waypoint_location.z_val, 5, vehicle_name=droneName)
            status = "MOVING"
            while not new.is_done():
                time.sleep(0.1)
                print("Drone is moving")
            status = "SEARCHING"
        else:
            print(f"Drone {droneName} is waiting for commands.")
            status = "WAITING"
            time.sleep(1)

        print("Drone {droneName} is ready to search at {current_target}")
        # if not command_queue.empty():
        #     command = command_queue.get()
        #     if command == "STOP":
        #         print(f"Drone {droneName} stopping.")
        #         status_queue.put((droneName, "STOPPED"))
        #         break  # Stop the process

        #     # Expecting a tuple (x, y, z)
        #     if isinstance(command, tuple) and len(command) == 3:
        #         x, y, z = command
        #         print(f"Drone {droneName} moving to ({x}, {y}, {z})")
        #         new_position = move_drone_relative(droneName, x, y, z, 5)
        #         status_queue.put((droneName, (new_position.x_val, new_position.y_val, new_position.z_val)))
        #         take_forward_picture(droneName, airsim.ImageType.Scene)


if __name__ == "__main__":
    print("This script is not meant to be run directly.")