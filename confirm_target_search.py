import airsim  # type: ignore
import time
import keyboard
import os
import numpy as np
import cv2
import math

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

# Arm the drone
client.armDisarm(True)
client.takeoffAsync().join()

altitude = -10      # Fixed altitude (negative for AirSim)
side_length = 10    # Square size
speed = 8           # Speed (m/s)
running = True      # Search loop
drone = "1"         # Drone

imgDir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'images')

if not os.path.exists(imgDir):
    os.makedirs(imgDir)

def take_forward_picture(drone_name, image_type):
    camera_name = "front-" + drone_name
    print(f"Taking picture from {camera_name}")
    response = client.simGetImage(camera_name=camera_name, image_type=image_type, vehicle_name=drone_name)
    
    filename = os.path.join("images", f"{camera_name}_scene_{image_type}")

    img1d = np.frombuffer(response, dtype=np.uint8)
    img_rgb = cv2.imdecode(img1d, cv2.IMREAD_COLOR)

    cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png

def confirm_target_search(client, center_x, center_y, side_length, altitude, speed):
    global running

    half_side = side_length / 2  

    # Waypoints for target to be centered
    square_corners = [
        (center_x - half_side, center_y - half_side),  # Bottom-left
        (center_x + half_side, center_y - half_side),  # Bottom-right
        (center_x + half_side, center_y + half_side),  # Top-right
        (center_x - half_side, center_y + half_side),  # Top-left
    ]

    while running:
        for x, y in square_corners:
            if not running:
                break

            # Move to the next corner
            client.moveToPositionAsync(
                x, y, altitude, speed,
                drivetrain=airsim.DrivetrainType.ForwardOnly,
                yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0)  
            ).join()

            # Delay for stabilization
            time.sleep(3)

            target_position = detect_brian()
            if target_position:
                # Yaw, pitch, and roll
                yaw_angle, pitch, roll = get_yaw_angle_to_target(client, target_position)
                client.rotateToYawAsync(yaw_angle).join() 
                client.moveByVelocityZAsync(0, 0, altitude, 1) 
            
            take_forward_picture(drone, airsim.ImageType.Scene)

            # Stop key 
            if keyboard.is_pressed('q'):
                print("Stop key pressed. Exiting search.")
                running = False
                break

def detect_brian():
    response = client.simGetObjectPose("OrangeBall")  # Replace with Brian
    
    if response.position:
        return response.position
    return None

def get_yaw_angle_to_target(client, target_position):
    drone_state = client.getMultirotorState()
    drone_position = drone_state.kinematics_estimated.position

    dx = target_position.x_val - drone_position.x_val
    dy = target_position.y_val - drone_position.y_val

    yaw = math.atan2(dy, dx)
    yaw_deg = math.degrees(yaw) % 360

    # Drone tilt
    pitch = -90     # Downward tilt
    roll = 45       # Sideways tilt

    return yaw_deg, pitch, roll

try:
    brian_position = detect_brian()
    
    if brian_position:
        center_x, center_y = brian_position.x_val, brian_position.y_val
        confirm_target_search(client, center_x, center_y, side_length, altitude, speed)
    else:
        print("Target not detected. Skipping search.")

finally:
    # Land the drone safely
    print("Landing")
    client.landAsync().join()

    # Check landed state and disarm
    landed_state = client.getMultirotorState().landed_state
    if landed_state == airsim.LandedState.Landed:
        client.armDisarm(False)
        print("Drone successfully disarmed.")
    else:
        print("Drone is not in a landed state. Skipping disarm.")

    # Disable API control
    client.enableApiControl(False)
    print("Flight operation completed.")