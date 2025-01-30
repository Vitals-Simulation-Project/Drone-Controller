from droneNode import globalVarSetup, takeOff
import airsim # type: ignore
import multiprocessing as mp
import random
import time




def singleDroneController(droneName, droneCount, queue):
    """ Drone process that listens for movement commands and executes them. """
    print(f"In single controller, Drone name: {droneName}")
    # globalVarSetup(droneCount, droneName)

    # Initialize AirSim client and take off
    client = takeOff(droneName)


    # moves the drone relative to its current position
    def move_drone_relative(drone_name, x, y, z, speed):
        state = client.getMultirotorState(vehicle_name=drone_name)
        current_position = state.kinematics_estimated.position
        #print("Current position: " + str(current_position))
        new_position = airsim.Vector3r(current_position.x_val + x, current_position.y_val + y, current_position.z_val + z)    
        return client.moveToPositionAsync(new_position.x_val, new_position.y_val, new_position.z_val, speed, vehicle_name=drone_name)

    
    while True:
        if not queue.empty():
            command = queue.get()
            if command == "STOP":
                print(f"Drone {droneName} stopping.")
                break  # Stop the process

            # Expecting a tuple (x, y, z)
            if isinstance(command, tuple) and len(command) == 3:
                x, y, z = command
                print(f"Drone {droneName} moving to ({x}, {y}, {z})")
                # client.moveToPositionAsync(x, y, z, velocity=5, vehicle_name=droneName)
                move_drone_relative(droneName, x, y, z, 5)

def parentController():
    """ Parent process to send commands to each drone """
    mp.set_start_method('spawn')  # Windows-specific start method

    drone_count = 5
    queues = {}  # Dictionary to store queues for each drone
    processes = []  # List to store process references

    # Create and start processes
    for x in range(drone_count):
        drone_name = str(x)
        queues[drone_name] = mp.Queue()
        p = mp.Process(target=singleDroneController, args=(drone_name, drone_count, queues[drone_name]))
        p.start()
        processes.append(p)

    # Parent process sending random movement commands
    try:
        while True:
            user_input = input("Enter 'rand' for random movement or 'STOP' to quit: ")
            
            if user_input == "STOP":
                for q in queues.values():
                    q.put("STOP")  # Stop all drones
                break
            
            elif user_input == "rand":
                for drone_name, q in queues.items():
                    x = random.uniform(-50, 50)
                    y = random.uniform(-50, 50)
                    z = random.uniform(-20, -5)  # Stay within flight limits
                    q.put((x, y, z))  # Send position command to drone
            else:
                # parse the input
                try:
                    x, y, z = map(float, user_input.split())
                    for q in queues.values():
                        q.put((x, y, z))
                except ValueError:
                    print("Invalid input. Type 'rand' for random movement or 'STOP' to exit.")
    
    except KeyboardInterrupt:
        for q in queues.values():
            q.put("STOP")

    # Wait for all processes to finish
    for p in processes:
        p.join()

if __name__ == '__main__':
    parentController()
