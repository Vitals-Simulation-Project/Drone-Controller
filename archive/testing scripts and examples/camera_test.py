import airsim # type: ignore
import os
import numpy as np
import cv2
import pprint
import time
import geopy.distance # type: ignore
import base64

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




def move_drone_relative(drone_name, x, y, z, speed):
    state = client.getMultirotorState(vehicle_name=drone_name)
    current_position = state.kinematics_estimated.position
    new_position = airsim.Vector3r(current_position.x_val + x, current_position.y_val + y, current_position.z_val + z)    
    client.moveToPositionAsync(new_position.x_val, new_position.y_val, new_position.z_val, speed, vehicle_name=drone_name).join()
    return new_position

drone = "0"

# directory to store pictures
imgDir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'images')

# create directory if it does not exist
if not os.path.exists(imgDir):
    os.makedirs(imgDir)

# set up client object to access multirotor drone
client = airsim.MultirotorClient()

# connect to AirSim simulator
client.confirmConnection()
client.enableApiControl(True, vehicle_name=drone)
client.armDisarm(True, vehicle_name=drone)

client.takeoffAsync(vehicle_name=drone).join()

# move up 20 meters
client.moveToZAsync(z=-20, velocity=8, vehicle_name=drone).join()

# hover
client.hoverAsync(vehicle_name=drone).join()


# wait 5 sec
time.sleep(5)


take_forward_picture(drone, airsim.ImageType.Scene)
take_forward_picture(drone, airsim.ImageType.Infrared)




# end connection
client.enableApiControl(False)
