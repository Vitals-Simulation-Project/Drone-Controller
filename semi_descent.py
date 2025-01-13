import airsim # type: ignore
import os
import numpy as np
import cv2
import pprint
import time

# directory to store pictures
imgDir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'images')

# create directory if it does not exist
if not os.path.exists(imgDir):
    os.makedirs(imgDir)

# set up client object to access multirotor drone
client = airsim.MultirotorClient()

# connect to AirSim simulator
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Async methods returns Future. Call join() to wait for task to complete.
client.takeoffAsync().join()
client.moveToPositionAsync(-10, 10, -10, 5).join()

client.moveToPositionAsync(10, 10, -10, 5).join()

# image collection loop
while True:

    # take images
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.DepthVis),
        airsim.ImageRequest("1", airsim.ImageType.DepthPlanar, True)])
    print('Retrieved images: %d', len(responses))

    # grab the current state of collision from the client (aka the drone)
    collision_info = client.simGetCollisionInfo()

    # stop if we encountered a collision
    if collision_info.has_collided:
        print("Collision at pos %s, normal %s, impact pt %s, penetration %f, name %s, obj id %d" % (
            pprint.pformat(collision_info.position),
            pprint.pformat(collision_info.normal),
            pprint.pformat(collision_info.impact_point),
            collision_info.penetration_depth, collision_info.object_name, collision_info.object_id))
        break

    time.sleep(0.1)

# Save images
for idx, response in enumerate(responses):
    filename = os.path.join(imgDir, str(idx))
    if response.pixels_as_float:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
        airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
    elif response.compress:  # Compressed PNG format
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
    else:  # Uncompressed array
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)  # Use frombuffer
        if img1d.size == response.height * response.width * 3:  # Check array size
            img_rgb = img1d.reshape(response.height, response.width, 3)
            cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb)  # Save as PNG
        else:
            print("Error: Image size mismatch. Skipping this image.")

# end connection
client.enableApiControl(False)
