import airsim  # type: ignore
import multiprocessing as mp
import random
import time
import os
import numpy as np
import cv2



LOCAL_IP = "172.20.11.22"


# make a global queue for images to process



# Enables api control, takes off drone, returns the client
def takeOff(droneName):
    client = airsim.MultirotorClient(LOCAL_IP)
    client.confirmConnection()
    client.enableApiControl(True, droneName)
    client.armDisarm(True, droneName)
    client.takeoffAsync(vehicle_name=droneName).join()

    print("Drone " + droneName + " is ready to fly")

    return client



def singleDroneController(droneName, droneCount, command_queue, status_queue):
    """ Drone process that listens for movement commands and sends status updates. """
    print(f"In single controller, Drone name: {droneName}")
    
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


    while True:
        if not command_queue.empty():
            command = command_queue.get()
            if command == "STOP":
                print(f"Drone {droneName} stopping.")
                status_queue.put((droneName, "STOPPED"))
                break  # Stop the process

            # Expecting a tuple (x, y, z)
            if isinstance(command, tuple) and len(command) == 3:
                x, y, z = command
                print(f"Drone {droneName} moving to ({x}, {y}, {z})")
                new_position = move_drone_relative(droneName, x, y, z, 5)
                status_queue.put((droneName, (new_position.x_val, new_position.y_val, new_position.z_val)))
                take_forward_picture(droneName, airsim.ImageType.Scene)


def parentController(drone_count):
    """ Parent process to send commands and receive status updates from drones. """
    mp.set_start_method('spawn')  # Windows-specific start method

    
    command_queues = {}  # Dictionary to store queues for sending commands
    status_queue = mp.Queue()  # Single queue for receiving updates
    processes = []  # List to store process references

    # Create and start processes
    for x in range(drone_count):
        drone_name = str(x)
        command_queues[drone_name] = mp.Queue()
        p = mp.Process(target=singleDroneController, args=(drone_name, drone_count, command_queues[drone_name], status_queue))
        p.start()
        processes.append(p)

    time.sleep(10)  # Wait for drones to take off


    try:
        while True:
            # Receive status updates from drones
            while not status_queue.empty():
                drone_name, status = status_queue.get()
                print(f"Update from Drone {drone_name}: {status}")

            user_input = input("Enter 'rand' for random movement or 'STOP' to quit: ")

            if user_input == "STOP":
                for q in command_queues.values():
                    q.put("STOP")  # Stop all drones
                break

            elif user_input == "rand":
                for drone_name, q in command_queues.items():
                    x = random.uniform(-50, 50)
                    y = random.uniform(-50, 50)
                    z = random.uniform(-20, -5)  # Stay within flight limits
                    q.put((x, y, z))  # Send position command to drone
            else:
                # Parse the input
                try:
                    x, y, z = map(float, user_input.split())
                    for q in command_queues.values():
                        q.put((x, y, z))
                except ValueError:
                    print("Invalid input. Type 'rand' for random movement or 'STOP' to exit.")

            time.sleep(1)  # Give some time for status updates
    
    except KeyboardInterrupt:
        for q in command_queues.values():
            q.put("STOP")

    # Wait for all processes to finish
    for p in processes:
        p.join()

if __name__ == '__main__':

    # directory to store pictures
    imgDir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'images')

    # create directory if it does not exist
    if not os.path.exists(imgDir):
        os.makedirs(imgDir)

    drone_count = 5
    parentController(drone_count)
