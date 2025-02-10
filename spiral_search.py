import airsim  # type: ignore
import math
import keyboard

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Takeoff
client.takeoffAsync().join()

center_x, center_y = 0, 0  # Center of the spiral
fixed_altitude = -10       # Fixed altitude (negative for AirSim)
radius_increment = 1       # Increase in radius per loop
angle_step = 10            # Angle step for smoother movement (degrees)
max_radius = 10            # Maximum radius for the spiral
speed = 10                 # Speed (m/s)

def spiral_search(client, center_x, center_y, fixed_altitude, radius_increment, angle_step, max_radius, speed):
    radius = 0
    angle = 0

    print("Starting sprial")
    while radius <= max_radius:
        # Calculate next waypoint
        x = center_x + radius * math.cos(math.radians(angle))
        y = center_y + radius * math.sin(math.radians(angle))

        # Fly to waypoint
        client.moveToPositionAsync(
            x, y, fixed_altitude, speed,
            drivetrain=airsim.DrivetrainType.ForwardOnly,  # Keep the drone moving forward
            yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0)  
        ).join()

        # Increment angle and radius for next waypoint
        angle += angle_step
        if angle >= 360:  
            angle -= 360
            radius += radius_increment  # Increase radius after each loop

        # Stop key during movement
        if keyboard.is_pressed('q'):
            print("Stop key pressed")
            running = False
            break


    print("Spiral complete")

try:
    spiral_search(client, center_x, center_y, fixed_altitude, radius_increment, angle_step, max_radius, speed)

except Exception as e:
    print(f"An error occurred: {e}")

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
        print("Drone is not in a landed state. Skipping disarm.")

    # Disable API control
    client.enableApiControl(False)
    print("Spiral search completed")