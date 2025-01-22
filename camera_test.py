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
client.moveToPositionAsync(-10, 10, -25, 5).join()


# responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.DepthVis, False, False)])
responses = client.simGetImages([
    airsim.ImageRequest(camera_name = "front-0", image_type=airsim.ImageType.DepthVis, pixels_as_float = False, compress = False),  #depth visualization image
    airsim.ImageRequest("front-0", airsim.ImageType.DepthPerspective, pixels_as_float = True, compress = True), #depth in perspective projection
    airsim.ImageRequest("front-0", airsim.ImageType.Scene, pixels_as_float = False, compress = True), #scene vision image in png format
    airsim.ImageRequest("front-0", airsim.ImageType.Scene, pixels_as_float = False, compress = False)])  #scene vision image in uncompressed RGBA array




for idx, response in enumerate(responses):

    filename = os.path.join("images", str(idx))
    print("Reponse number: " + str(idx))

    if response.pixels_as_float:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
        airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
    elif response.compress: #png format
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
    else: #uncompressed array
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8) # get numpy array
        img_rgb = img1d.reshape(response.height, response.width, 3) # reshape array to 4 channel image array H X W X 3
        cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png


# # get numpy array
# img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8) 

# # reshape array to 4 channel image array H X W X 4
# img_rgb = img1d.reshape(response.height, response.width, 3)

# # original image is fliped vertically
# # img_rgb = np.flipud(img_rgb)

# # write to png 
# airsim.write_png(os.path.normpath("images/test" + '.png'), img_rgb) 

# end connection
client.enableApiControl(False)
