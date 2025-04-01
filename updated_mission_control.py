#======================#
#  mission_control.py  #
#======================#

# Launches the Vitals Simulation project
# - Initializes Image Directories
# - Spawns All Necessary Processes
# - Handles Shutdown Event (Keyboard Interrupt, StopSimulation)

# Python Dependencies
import multiprocessing as mp
import os

# Project Imports
import local_config
import parent_controller
import websocket.websocket_server

# Constants
DRONE_COUNT = 1 # Number of drones to spawn
processes = {}

def initialize_image_directory():
    '''Set up the images directory to store pictures'''
    # Set directory to store pictures
    img_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'images')

    if not os.path.exists(img_dir):
        os.makedirs(img_dir)

    # Create image subdirectory for each drone
    for drone_id in range(DRONE_COUNT):
        drone_dir = os.path.join(img_dir, f"drone_{drone_id}")
         
        if os.path.exists(drone_dir):
            for file in os.listdir(drone_dir):
                file_path = os.path.join(drone_dir, file)
                
                if os.path.isfile(file_path):
                    os.remove(file_path)  # Remove file

            os.rmdir(drone_dir)

        if not os.path.exists(drone_dir):
            os.makedirs(drone_dir)

def spawn_processes():
    '''Creates processes and returns process as a map'''

    global processes

    processes = {
        "parent_controller": mp.Process(target=parent_controller.start_parent_controller, args=(DRONE_COUNT, shutdown_event)),
        "websocket_server": mp.Process(target=websocket.websocket_server.start_websocket_server, daemon=False)
    }

    # for drone_id in range (DRONE_COUNT):
    #     processes[f"drone_{drone_id}"] = mp.Process(target=parent_controller.start_parent_controller, args=(DRONE_COUNT, shutdown_event), daemon=False)

def terminate_processes():
    '''Terminates all Processes'''

    global processes

    for process in processes.values():
        if process and process.is_alive():
            process.terminate()
            process.join()
    

# MAIN
if __name__ == "__main__":

    initialize_image_directory()

    mp.set_start_method('spawn') # Windows Specific
    shutdown_event = mp.Event() # Event called to shutdown program

    spawn_processes()

    try:

        for process in processes.values():
            process.start()

        while not shutdown_event.is_set():
            ...

    except:
        shutdown_event.set()

    finally:
        print("Terminating Processes...")

        terminate_processes()