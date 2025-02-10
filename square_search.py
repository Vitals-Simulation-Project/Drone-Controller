import airsim  # type: ignore
import time
import keyboard

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Takeoff
client.takeoffAsync().join()

side_length = 10  # Length of square (meters)
altitude = -10    # Fixed altitude (negative for AirSim)
speed = 6         # Speed (m/s)

# Define the square to starting position
starting_x, starting_y = 0, 0
square_corners = [
    (starting_x, starting_y),  # Corner 1
    (starting_x + side_length, starting_y),  # Corner 2
    (starting_x + side_length, starting_y + side_length),  # Corner 3
    (starting_x, starting_y + side_length),  # Corner 4
]

running = True

def square_search(client, corners, altitude, speed):
    global running
    print("Starting square search")
    try:
        while running:
            for corner in corners:
                if not running: # Check at each corner
                    break
                x, y = corner

                # Move to the next corner
                client.moveToPositionAsync(
                    x, y, altitude, speed,
                    drivetrain=airsim.DrivetrainType.ForwardOnly,
                    yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0) 
                ).join()

                # Add a delay for stabilization
                time.sleep(1)

                # Stop key during movement
                if keyboard.is_pressed('q'):
                    print("Stop key pressed")
                    running = False
                    break

    except Exception as e:
        print(f"An error occurred: {e}")
                
try:
    square_search(client, square_corners, altitude, speed)

finally:
    # Land
    print("Landing")
    client.landAsync().join()

    # Check landed state
    landed_state = client.getMultirotorState().landed_state
    if landed_state == airsim.LandedState.Landed:
        client.armDisarm(False)
        print("Drone successfully disarmed")
    else:
        print("Drone is not in a landed state. Skipping disarm")

    # Disable API control
    client.enableApiControl(False)
    print("Flight operation completed")