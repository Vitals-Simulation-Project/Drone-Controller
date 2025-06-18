import airsim;
import random
import glob
import time
import cv2
import numpy as np
import time
import os
import shutil
import base64
import sys
import math
import time
import argparse
import pprint
import numpy
from airsim.types import Pose,Quaternionr,Vector3r


drone = airsim.MultirotorClient() 
drone.confirmConnection()
drone.enableApiControl(True,vehicle_name="0")
drone.armDisarm(True,vehicle_name="0")
drone.takeoffAsync(vehicle_name="0")
drone.getMultirotorState(vehicle_name="0")


##get drone
##getGpsData()
##format the data
##get Image or thermal or what ever it is /// convert it to base 64 
##imageRequest()
## can research if i can pass a path

## take all this data and create a data struct
## pass the data struct to the AI

image_list = drone.simGetImages([airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False), 
                                                    airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False)])



test_image  = base64.b64encode(image_list[0].image_data_uint8)



print(drone.getGpsData()) ## we can also pull time taken if we want but there is a lot of ways to do that also can get velocity idk what we want so this is just a test rn
print(drone.getDistanceSensorData(distance_sensor_name='Distance'))
print(drone.getDistanceSensorData(distance_sensor_name='Distance2'))

print("GPS =  Latitude (%.10f) , Longitude(%.10f) \n HightDistanceSensor = %i \n FrontDistanceSensor = %i" %(drone.getGpsData().gnss.geo_point.latitude , drone.getGpsData().gnss.geo_point.longitude ,drone.getDistanceSensorData(distance_sensor_name='Distance').distance, drone.getDistanceSensorData(distance_sensor_name='Distance2').distance))
 ## need something better than floats for lat and long its not long enough or accurate 
"""for i in range(len(image_list)):
    test_image  = base64.b64encode(image_list[i].image_data_uint8)
    drone.getGpsData()
    drone.getDistanceSensorData(distance_sensor_name='Distance')
    drone.getDistanceSensorData(distance_sensor_name='Distance2')
    #format and send"""

if(drone.getDistanceSensorData(distance_sensor_name='Distance').distance<15):
    print("im here")

    ##drone.moveToPositionAsync(15,15,15,3, vehicle_name="0").join()
    drone.moveToZAsync(-20,3).join()
    drone.hoverAsync().join()
    print("moving")
    
    

    

## format all this data later and create what exactly we want to send the AI


##if getDistanceSensorData().distance < 15: ## can do something like this is the bottom sesnor( height
    ## STOP EVERYTHING AND PULL THE FUCK UP
## if getDistanceSensorData2().distance < 3 ## forward sensor , 
    ## stop ALL MOMENTUM UR GUNNA CRASH



##print(base64.b64encode(image_list[0].image_data_float)) ##this one breaks
##print(drone.simGetImages([airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False), 
                                                    ##airsim.ImageRequest("front_center", airsim.ImageType.Scene, False, False)]))


print("done")


"""for i in range(1,5):
            lidarData = drone.getLidarData()
            if (len(lidarData.point_cloud) < 3):
                print("\tNo points received from Lidar data")
            else:
                points = drone.parse_lidarData(lidarData)
                print("\tReading %d: time_stamp: %d number_of_points: %d" % (i, lidarData.time_stamp, len(points)))
                print("\t\tlidar position: %s" % (pprint.pformat(lidarData.pose.position)))
                print("\t\tlidar orientation: %s" % (pprint.pformat(lidarData.pose.orientation)))
            time.sleep(5)

def parse_lidarData(self, data):

    # reshape array of floats to array of [X,Y,Z]
    points = numpy.array(data.point_cloud, dtype=numpy.dtype('f4'))
    points = numpy.reshape(points, (int(points.shape[0]/3), 3))
       
    return points"""
