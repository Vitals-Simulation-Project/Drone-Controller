import airsim # type: ignore
import os
import numpy as np
import cv2
import pprint
import time


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

# Async methods returns Future. Call join() to wait for task to complete.
client.takeoffAsync(vehicle_name=drone).join()
client.moveToPositionAsync(-10, 10, -100, 5).join()


responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
response = responses[0]

# get numpy array
img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8) 

# reshape array to 4 channel image array H X W X 4
img_rgb = img1d.reshape(response.height, response.width, 3)

# original image is fliped vertically
# img_rgb = np.flipud(img_rgb)

# write to png 
airsim.write_png(os.path.normpath("images/test" + '.png'), img_rgb) 

# end connection
client.enableApiControl(False)
