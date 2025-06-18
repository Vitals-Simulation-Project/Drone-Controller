import airsim #type: ignore
import numpy as np
import os
import tempfile
import cv2
import time
import math

drone = airsim.MultirotorClient()
drone.confirmConnection()
drone.enableApiControl(True)
drone.armDisarm(True)

drone.takeoffAsync().join()



def MoveToGPSAsyncCollsion(self, latitude, longitude, altitude, velocity, timeout_sec = 3e+38,  vehicle_name = ''):
    self.moveToGPSAsync(latitude,longitude,altitude,velocity,vehicle_name= vehicle_name)
    t_end = time.time() + 30
    while (time.time()<t_end): ## need a solution to this its just a bandaid rn 
    ##print("im in here")
        if (self.getDistanceSensorData(distance_sensor_name='Distance').distance<5 or self.getDistanceSensorData(distance_sensor_name='Distance2').distance<5):
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
    drone.hoverAsync()
    print("im here 1")
    responses = drone.simGetImages([airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False),airsim.ImageRequest("front_center", airsim.ImageType.Infrared,False,False)])
    TEST_response= drone.simGetImage(camera_name="front_center", image_type= airsim.ImageType.Infrared, vehicle_name='0')
    ##response = client.simGetImage(camera_name=camera_name, image_type=image_type, vehicle_name=drone_name)
    ##responcess = drone.simGetImages(airsim.ImageRequest("front_center", airsim.ImageType.Infrared, True)) 
    depth = responses[0]
    print("im here 2")

    depth_img_in_meters = airsim.list_to_2d_float_array(responses[0].image_data_float, responses[0].width, responses[0].height) 

    depth_img_in_meters = depth_img_in_meters.reshape(responses[0].height, responses[0].width, 1)
    depth_8bit_lerped = np.interp(depth_img_in_meters, (0, 100), (0, 255))

    
    ## change this to coresponding spot i want with the map but like for now testing is fine
    ##airsim.list_to_2d_float_array(responses[1].image_data_float, responses[1].width, responses[1].height) ## i think there is something i can do with this to get color to read
    print("im here 4")
    """depth_img_in_meters = airsim.list_to_2d_float_array(responses[0].image_data_float, responses[0].width, responses[0].height) 

    depth_img_in_meters = depth_img_in_meters.reshape(responses[0].height, responses[0].width, 1)


    # Lerp 0..100m to 0..255 gray values
    depth_8bit_lerped = np.interp(depth_img_in_meters, (0, 100), (0, 255))
    cv2.imwrite("depth_visualization.png", depth_8bit_lerped.astype('uint8'))"""
    ##calculations = math.sqrt(distance**2 - height**2) ## this needs jesus something about a deprecation warning and domain error 
    print("im here 4.5")
    ##img = responses[1]
    img = TEST_response
    ##camera_name = "front_center"
    ##filename = os.path.join("infared", f"{camera_name}_scene_{airsim.ImageType.Infrared}")

   
    img1d = np.frombuffer(img, dtype=np.uint8) ## breaking here rn 
    img_rgb = cv2.imdecode(img1d, cv2.IMREAD_COLOR)
    ##cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png
    ##img = cv2.imread(filename)
    hsv = cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)
    print("im here 5")

    ##img.resize(256,144) ## width , height ## this is breaking after the changes/ cant do it 


    lower = np.array([0,0,0], dtype = "uint8")
    upper = np.array([192,192,192], dtype = "uint8")


    mask = cv2.inRange(hsv, lower, upper)
    print("im here 6")
    ##cv2.imshow("Mask",mask)
    ##cv2.imshow("img",img1d)
    ##cv2.waitKey(0)
    
    
    print("im here 7")

    height= drone.getDistanceSensorData(distance_sensor_name='Distance').distance
    print("im here 3")

    

    
    perpixel = 90/256 ## might  be wrong there is a lot of things to check but in theroy if everything is right it works 
    horizontal = np.argwhere(mask)[5][0] ## if its not working this 0 is a 1 # need error handling here if there is nothing
    vert = np.argwhere(mask)[5][1]
    distance = depth_img_in_meters[horizontal][vert][0] ## could be back wards
    print(distance)
    print(height)
    calculations = math.sqrt(distance**2 - height**2) ## need a catch for this i think 
    horizontalangle = perpixel * horizontal
    return horizontalangle , calculations
    ##return "test"

MoveToGPSAsyncCollsion(drone, 0,0,5,3,vehicle_name='0')

numbers = calculateDistance_angle()
print(numbers)
state = drone.getMultirotorState()
currentposition = state.kinematics_estimated.position
newY= currentposition.y_val - (math.sin(numbers[0]) * numbers[1] )
newX= currentposition.x_val - (math.cos(numbers[0]) * numbers[1] )
drone.moveToPositionAsync(newY,newX , currentposition.z_val ,3, vehicle_name= '0') ## problem im now noticing is idk if i need to add or subtract since i know know what direction the guy is in 

print("done")
time.sleep(10)
print("double done")
## calcualte next place and go 