import airsim  # type: ignore
import time
import keyboard

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Takeoff
client.takeoffAsync().join()

center_x, center_y = 0, 0  # Center point of the search
initial_side_length = 5     # Initial square size (meters)
side_increment = 5          # How much the square expands per loop
max_side_length = 30        # Maximum square size
altitude = -10              # Fixed altitude (negative for AirSim)
speed = 6                   # Speed (m/s)

running = True

def expanding_square_search(client, center_x, center_y, initial_side_length, side_increment, max_side_length, altitude, speed):
    global running
    side_length = initial_side_length 

    print("Starting expanding square search")

    try:
        while running and side_length <= max_side_length:
            # Define corners based on the center and current side length
            half_side = side_length / 2
            square_corners = [
                (center_x - half_side, center_y - half_side),  # Bottom-left
                (center_x + half_side, center_y - half_side),  # Bottom-right
                (center_x + half_side, center_y + half_side),  # Top-right
                (center_x - half_side, center_y + half_side),  # Top-left
            ]

            for corner in square_corners:
                if not running:  # Check at each corner
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

            # Expand the square for the next iteration
            side_length += side_increment

    except Exception as e:
        print(f"An error occurred: {e}")

try:
    expanding_square_search(client, center_x, center_y, initial_side_length, side_increment, max_side_length, altitude, speed)

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