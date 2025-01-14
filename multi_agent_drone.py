import airsim # type: ignore
import cv2
import numpy as np
import os
import pprint
import tempfile

# Use below in settings.json with Blocks environment
"""
{
	"SeeDocsAt": "https://github.com/Microsoft/AirSim/blob/main/docs/settings.md",
	"SettingsVersion": 1.2,
	"SimMode": "Multirotor",
	"ClockSpeed": 1,
	
	"Vehicles": {
		"Drone1": {
		  "VehicleType": "SimpleFlight",
		  "X": 4, "Y": 0, "Z": -2
		},
		"Drone2": {
		  "VehicleType": "SimpleFlight",
		  "X": 8, "Y": 0, "Z": -2
		}

    }
}
"""

# directory to store pictures
imgDir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'images')

# create directory if it does not exist
if not os.path.exists(imgDir):
    os.makedirs(imgDir)


# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, "0")
client.enableApiControl(True, "1")
client.armDisarm(True, "0")
client.armDisarm(True, "1")

airsim.wait_key('Press any key to takeoff')
f1 = client.takeoffAsync(vehicle_name="0")
f2 = client.takeoffAsync(vehicle_name="1")
f1.join()
f2.join()

state1 = client.getMultirotorState(vehicle_name="0")
s = pprint.pformat(state1)
print("state: %s" % s)
state2 = client.getMultirotorState(vehicle_name="1")
s = pprint.pformat(state2)
print("state: %s" % s)

airsim.wait_key('Press any key to move vehicles')
f1 = client.moveToPositionAsync(-5, 5, -10, 5, vehicle_name="0")
f2 = client.moveToPositionAsync(5, -5, -10, 5, vehicle_name="1")
f1.join()
f2.join()

airsim.wait_key('Press any key to take images')
# get camera images from the car
responses1 = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
    airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)], vehicle_name="0")  #scene vision image in uncompressed RGB array
print('Drone1: Retrieved images: %d' % len(responses1))
responses2 = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
    airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)], vehicle_name="1")  #scene vision image in uncompressed RGB array
print('Drone2: Retrieved images: %d' % len(responses2))

print ("Saving images to %s" % imgDir)



for idx, response in enumerate(responses1 + responses2):

    filename = os.path.join(imgDir, str(idx))

    if response.pixels_as_float:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
        airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
    elif response.compress: #png format
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
    else: #uncompressed array
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8) #get numpy array
        img_rgb = img1d.reshape(response.height, response.width, 3) #reshape array to 3 channel image array H X W X 3
        cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png

airsim.wait_key('Press any key to reset to original state')

client.armDisarm(False, "0")
client.armDisarm(False, "1")
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False, "0")
client.enableApiControl(False, "1")


