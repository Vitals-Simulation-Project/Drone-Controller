import airsim;
import numpy as np
import os
import tempfile
import cv2
import time

drone = airsim.MultirotorClient()
import math
def MoveToGPSAsyncCollsion(self, latitude, longitude, altitude, velocity, timeout_sec = 3e+38,  vehicle_name = ''):
    
    self.moveToGPSAsync(latitude,longitude,altitude,velocity,vehicle_name= vehicle_name)
    t_end = time.time() + 30
    while (time.time()<t_end): ## need a solution to this its just a bandaid rn 
    ##print("im in here")
        if (self.getDistanceSensorData(distance_sensor_name='Distance').distance<15 or self.getDistanceSensorData(distance_sensor_name='Distance2').distance<15):
            ##drone.moveToPositionAsync(15,15,15,3, vehicle_name="0").join()
        
            ##drone.moveToZAsync(-20,3).join()
            gpsData = self.getGpsData().gnss.geo_point
        
            self.moveToGPSAsync(gpsData.latitude,gpsData.latitude,gpsData.altitude+10,3).join()
            self.moveToGPSAsync(latitude,longitude,altitude,velocity,vehicle_name= vehicle_name)
            ##drone.hoverAsync()
            print("moving up")
    ##if(drone.getDistanceSensorData(distance_sensor_name='Distance2').distance<15):
        ##print("BREAK!!!!!!!!!")
##drone.moveToPositionAsync(10,-00,-30,5,vehicle_name="0").join()
    self.hoverAsync



def calculateDistance_angle():
    responses = drone.simGetImages([airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False), airsim.ImageType.Infrared, True, False]) 
    depth = responses[0]

    depth_img_in_meters = airsim.list_to_2d_float_array(responses[0].image_data_float, responses[0].width, responses[0].height) 

    depth_img_in_meters = depth_img_in_meters.reshape(responses[0].height, responses[0].width, 1)
    depth_8bit_lerped = np.interp(depth_img_in_meters, (0, 100), (0, 255))

    
    height= drone.getDistanceSensorData(distance_sensor_name='Distance').distance

    distance = depth_img_in_meters[64][36]

    calculations = math.sqrt(distance**2 - height**2)
    img = responses[1]
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

    img.resize(256,144) ## width , height


    lower = np.array([0,0,0], dtype = "uint8")
    upper = np.array([192,192,192], dtype = "uint8")

    mask = cv2.inRange(hsv, lower, upper)


    

    perpixel = 90/128
    horizontal = np.argwhere(mask)[5][0] ## if its not working this 0 is a 1
    horizontalangle = perpixel * horizontal
    return horizontalangle , calculations
    ## take 