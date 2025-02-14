import airsim # type: ignore
import os
import numpy as np
import cv2
import pprint
import time



def take_forward_picture(drone_name, image_type):
    camera_name = "front-" + drone_name
    print(f"Taking picture from {camera_name}")
    response = client.simGetImage(camera_name=camera_name, image_type=image_type, vehicle_name=drone_name)
    
    filename = os.path.join("images", f"{camera_name}_scene_{image_type}")

    img1d = np.frombuffer(response, dtype=np.uint8)
    img_rgb = cv2.imdecode(img1d, cv2.IMREAD_COLOR)

    cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png

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

# Async methods returns Future. Call join() to wait for task to complete.
client.takeoffAsync(vehicle_name=drone).join()
move_drone_relative(drone, 150, 0, -60, 5)
move_drone_relative(drone, 0, 50, 40, 5)

take_forward_picture(drone, airsim.ImageType.Scene)
take_forward_picture(drone, airsim.ImageType.Infrared)


# # responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.DepthVis, False, False)])
# responses = client.simGetImages([
#     airsim.ImageRequest(camera_name = "front-0", image_type=airsim.ImageType.DepthVis, pixels_as_float = False, compress = False),  #depth visualization image
#     airsim.ImageRequest(camera_name = "front-0", image_type=airsim.ImageType.Segmentation, pixels_as_float = False, compress = False), 
#     airsim.ImageRequest(camera_name = "front-0", image_type=airsim.ImageType.Infrared, pixels_as_float = False, compress = False), # infrared image
#     airsim.ImageRequest(camera_name = "front-0", image_type=airsim.ImageType.Scene, pixels_as_float = False, compress = False)])  #scene vision image in uncompressed RGBA array

# print('Retrieved images: %d' % len(responses))


# for idx, response in enumerate(responses):

#     filename = os.path.join("images", str(idx))
#     print("Reponse number: " + str(idx))

#     if response.pixels_as_float:
#         print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
#         airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
#     elif response.compress: #png format
#         print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
#         airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
#     else: #uncompressed array
#         print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
#         img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8) # get numpy array
#         img_rgb = img1d.reshape(response.height, response.width, 3) # reshape array to 4 channel image array H X W X 3
#         cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png


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
