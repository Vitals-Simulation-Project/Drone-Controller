import airsim;
import numpy as np
import os
import tempfile
import cv2
import time
import math

drone = airsim.MultirotorClient()
drone.confirmConnection()
##drone.enableApiControl(True,vehicle_name="0")
##drone.armDisarm(True,vehicle_name="0")
print("im here 1")


"""drone.takeoffAsync(vehicle_name="0")
print("im here 2")
##drone.moveToZAsync(-20,3).join()
##drone.hoverAsync().join()
drone.hoverAsync(vehicle_name='0')
print("im here 2.5")
##drone.moveToPositionAsync(0,0,400,5,vehicle_name="0").join()
##drone.getMultirotorState(vehicle_name="0")
drone.moveToPositionAsync(-50,-50,-50,5, vehicle_name="0")
##f1= drone.moveToGPSAsync(100,500,300,5,vehicle_name= "0")
print("im here 3")

print("im here 3.5")"""

drone = airsim.MultirotorClient()
drone.confirmConnection()
drone.enableApiControl(True)
drone.armDisarm(True)

drone.takeoffAsync().join()

##drone.moveToZAsync(-5, 5).join()
drone.hoverAsync().join()
##drone.moveToPositionAsync(-100,-100,-10,5,vehicle_name="0").join()
##drone.hoverAsync

drone.moveToPositionAsync(0,0,-10,5,vehicle_name="0").join()
drone.hoverAsync().join()

"""t_end = time.time() + 30
while (time.time()<t_end):
    print("im in here")
    if(drone.getDistanceSensorData(distance_sensor_name='Distance').distance<15):
    ##drone.moveToPositionAsync(15,15,15,3, vehicle_name="0").join()
        drone.moveToZAsync(-20,3).join()
        drone.hoverAsync().join()
        print("moving")
    if(drone.getDistanceSensorData(distance_sensor_name='Distance2').distance<15):
        print("BREAK!!!!!!!!!")"""


##image_list = drone.simGetImages(airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True,False))

responses = drone.simGetImages([airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False)]) 
response= responses[0]
print("im here 4")
"""depth = np.array(response.image_data_float, dtype=np.float32)
depth = depth.reshape(response.height, response.width)
depth = np.array(depth * 255, dtype=np.uint8)
##cv2.imwrite('depth.png', depth)

cv2.imwrite(os.path.normpath('depth.png'), depth) # write to png
"""


                                               
depth_img_in_meters = airsim.list_to_2d_float_array(responses[0].image_data_float, responses[0].width, responses[0].height) 

depth_img_in_meters = depth_img_in_meters.reshape(responses[0].height, responses[0].width, 1)


# Lerp 0..100m to 0..255 gray values
depth_8bit_lerped = np.interp(depth_img_in_meters, (0, 100), (0, 255))
cv2.imwrite("depth_visualization.png", depth_8bit_lerped.astype('uint8'))

# Convert depth_img to millimeters to fill out 16bit unsigned int space (0..65535). Also clamp large values (e.g. SkyDome) to 65535
depth_img_in_millimeters = depth_img_in_meters * 1000
depth_16bit = np.clip(depth_img_in_millimeters, 0, 65535)
cv2.imwrite("depth_16bit.png", depth_16bit.astype('uint16'))
print("im here 5")

print(response.width)
print(response.height)

print(depth_img_in_meters[64][36])
print(depth_img_in_millimeters[64][36]/1000)
print(drone.getDistanceSensorData(distance_sensor_name='Distance2'))

height= drone.getDistanceSensorData(distance_sensor_name='Distance').distance
print(height)
distance = depth_img_in_meters[64][36]
print(distance)
calculations = math.sqrt(distance**2 - height**2)
print(calculations)
perpixel = 90/128
horizontalangle = perpixel * 64

"""tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")
print ("Saving images to %s" % tmp_dir)
try:
    os.makedirs(tmp_dir)
except OSError:
    if not os.path.isdir(tmp_dir):
        raise

for idx, response in enumerate(responses):

    filename = os.path.join(tmp_dir, str(idx))

    if response.pixels_as_float:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
        airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
    elif response.compress: #png format
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
    else: #uncompressed array
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) # get numpy array
        img_rgb = img1d.reshape(response.height, response.width, 3) # reshape array to 4 channel image array H X W X 3
        cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png"""




##drone.reset()
##drone.armDisarm(False)

# that's enough fun for now. let's quit cleanly
##drone.enableApiControl(False)

##drone.hoverAsync().join()
##drone.moveToPositionAsync(5,0,-5,1,vehicle_name="0").join()
drone.hoverAsync().join()
print("done")
drone.enableApiControl(False)