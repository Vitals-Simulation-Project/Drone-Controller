import airsim
import numpy as np
import os
import tempfile
import cv2
import time
import Put_it_all_together 
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
        if (self.getDistanceSensorData(distance_sensor_name='Distance').distance<5 or self.getDistanceSensorData(distance_sensor_name='Distance2').distance<5):
            gpsData = self.getGpsData().gnss.geo_point
        
            self.moveToGPSAsync(gpsData.latitude,gpsData.latitude,gpsData.altitude+10,3).join()
            self.moveToGPSAsync(latitude,longitude,altitude,velocity,vehicle_name= vehicle_name)
            ##print("moving up")
    self.hoverAsync

def calculateDistance_angle():
    drone.hoverAsync()
    #print("im here 1")
    responses = drone.simGetImages([airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False),airsim.ImageRequest("front_center", airsim.ImageType.Infrared,False,False)])
    TEST_response= drone.simGetImage(camera_name="front_center", image_type= airsim.ImageType.Infrared, vehicle_name='0') ## throws a fit if i do them both in one call
   
    ##depth = responses[0]
    ##print("im here 2")

    depth_img_in_meters = airsim.list_to_2d_float_array(responses[0].image_data_float, responses[0].width, responses[0].height) 

    depth_img_in_meters = depth_img_in_meters.reshape(responses[0].height, responses[0].width, 1)
    depth_8bit_lerped = np.interp(depth_img_in_meters, (0, 100), (0, 255))

    ##print(responses[0].width) #256
    ##print("width^^^^^")

    
   
  
    ##img = responses[1]
    img = TEST_response
  

   
    img1d = np.frombuffer(img, dtype=np.uint8) ## breaking here rn 
    img_rgb = cv2.imdecode(img1d, cv2.IMREAD_COLOR)
    hsv = cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)
    ##print("im here 5")

    ##img.resize(256,144) ## width , height ## this is breaking after the changes/ cant do it / might need this when integrating everyones code


    lower = np.array([0,0,0], dtype = "uint8")
    upper = np.array([192,192,192], dtype = "uint8")


    mask = cv2.inRange(hsv, lower, upper)
    #print("im here 6")
    ##cv2.imshow("Mask",mask)
    ##cv2.imshow("img",img1d)
    ##cv2.waitKey(0)
    ##mask.resize(256,288)
    
    
    
    ##print("im here 7")

    height= drone.getDistanceSensorData(distance_sensor_name='Distance').distance
    ##print("im here 3")

    
    
    perpixel = 90/256 ## might  be wrong there is a lot of things to check but in theroy if everything is right it works #half of the big angle 
    horizontal = np.argwhere(mask)[5][1] ## if its not working this 0 is a 1 # need error handling here if there is nothing
    
    if(horizontal >= (256/2)):
        #print("right")
        clockwise = True
    elif(horizontal<(256/2)):
        #print("left")
        clockwise = False   
    vert = np.argwhere(mask)[5][0]
    distance = depth_img_in_meters[vert][horizontal][0] ## could be back wards
    #print(horizontal)
    #print("horizontal ^^^")
    #print(vert)
    #print("vert ^^^^")
    ##print(height)
    calculations = math.sqrt(distance**2 - height**2) ## need a catch for this i think 
    horizontalangle = perpixel * horizontal
    #time.sleep(30)
    return horizontalangle , calculations , clockwise
    ##return "test"

def to_eularian_angles(q):
    z = q.z_val
    y = q.y_val
    x = q.x_val
    w = q.w_val
    ysqr = y * y

    # roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    roll = math.atan2(t0, t1)

    # pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = max(min(t2, 1.0), -1.0)  # Clamp t2 to the range [-1, 1]
    pitch = math.asin(t2)

    # yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    yaw = math.atan2(t3, t4)

    yaw = yaw * (180/ math.pi)

    return (roll, pitch, yaw)  # Standard order: roll, pitch, yaw

def CordCalulcation(angle, distance,clockwise):
    ## bearing = drone. get its angle from north
    
    state = drone.getMultirotorState()
    q = state.kinematics_estimated.orientation
    roll , pitch, yaw = to_eularian_angles(q)
    #print(yaw)
    #print("yaw^^^")
    if (clockwise == True):
        yaw = yaw + angle
    elif(clockwise == False):
        yaw = yaw - angle
    
    
    print(yaw)
    currentposition = state.kinematics_estimated.position
    if(yaw >0  and yaw <= 90 ):
       # print("quad 1")
        newY= currentposition.y_val + math.sin(math.radians(angle)) * distance
        newX= currentposition.x_val + (math.cos(math.radians(angle)) * distance )
    elif(yaw >90  and yaw <= 180):
       # print("quad 2")
        newY= currentposition.y_val +math.sin(math.radians(angle)) * distance
        newX= currentposition.x_val - (math.cos(math.radians(angle)) * distance )
    elif(yaw >-180  and yaw <= -90):
        #print("quad 3")
        newY= currentposition.y_val -math.sin(math.radians(angle)) * distance
        newX= currentposition.x_val - (math.cos(math.radians(angle)) * distance )
    elif(yaw >-90  and yaw <= 0):
        #print("quad 4")
        newY= currentposition.y_val -math.sin(math.radians(angle)) * distance
        newX= currentposition.x_val + (math.cos(math.radians(angle)) * distance )
   
    
    return newX,newY, currentposition.z_val


"""state = drone.getMultirotorState()
currentposition = state.kinematics_estimated.position
drone.moveToPositionAsync(currentposition.x_val,currentposition.y_val+10 , currentposition.z_val-3 ,3, vehicle_name= '0').join() 
calculateDistance_angle()
drone.rotateToYawAsync(180).join()
calculateDistance_angle()"""
##MoveToGPSAsyncCollsion(drone, 0,0,5,3,vehicle_name='0') ##34+45
"""state = drone.getMultirotorState()
currentposition = state.kinematics_estimated.position

drone.moveToPositionAsync(currentposition.x_val,currentposition.y_val+1 , currentposition.z_val-10 ,3, vehicle_name= '0').join() 
#drone.moveToPositionAsync(currentposition.x_val,currentposition.y_val+10 , currentposition.z_val-5 ,3, vehicle_name= '0').join() #22 + 45
angle , distance, clockwise =  calculateDistance_angle()
print(angle, distance, clockwise)
newX,newY,newZ = CordCalulcation(angle, distance,clockwise)
print(newX , newY,newZ)
drone.moveToPositionAsync(newX , newY, newZ -5 , 3 , vehicle_name='0').join()

drone.rotateToYawAsync(270).join()
angle , distance, clockwise =  calculateDistance_angle()
print(angle, distance, clockwise)
newX,newY,newZ = CordCalulcation(angle, distance,clockwise)
print(newX , newY,newZ)
drone.moveToPositionAsync(newX , newY, newZ -5 , 3 , vehicle_name='0').join()"""


##drone.moveToPositionAsync(currentposition.x_val,currentposition.y_val+10 , currentposition.z_val-5 ,3, vehicle_name= '0').join() #22 + 45
##print(calculateDistance_angle())

def testcase1():
    state = drone.getMultirotorState()
    currentposition = state.kinematics_estimated.position
    
    ## first position
    print("1st position ")
    drone.moveToPositionAsync(currentposition.x_val,currentposition.y_val+10 , currentposition.z_val-5 ,3, vehicle_name= '0').join() #22 + 45
    angle , distance, clockwise =  calculateDistance_angle()
    print(angle, distance, clockwise)
    newX,newY,newZ = CordCalulcation(angle, distance,clockwise)
    print(newX , newY,newZ)
    drone.moveToPositionAsync(newX , newY, newZ  , 3 , vehicle_name='0').join()
    
    ## rotate , second position
    print("2st position ")
    drone.rotateToYawAsync(90).join()
    time.sleep(1)
    angle , distance, clockwise =  calculateDistance_angle()
    print(angle, distance, clockwise)
    newX,newY,newZ = CordCalulcation(angle, distance,clockwise)
    print(newX , newY,newZ)
    drone.moveToPositionAsync(newX , newY, newZ  , 3 , vehicle_name='0').join()

    #rotate, 3rd postions
    print("3st position ")
    drone.rotateToYawAsync(270).join()
    time.sleep(1)
    angle , distance, clockwise =  calculateDistance_angle()
    print(angle, distance, clockwise)
    newX,newY,newZ = CordCalulcation(angle, distance,clockwise)
    print(newX , newY,newZ)
    drone.moveToPositionAsync(newX , newY, newZ  , 3 , vehicle_name='0').join()

    ##rotate , 4th position 
    print("4st position ")
    drone.rotateToYawAsync(180).join()
    time.sleep(1)
    angle , distance, clockwise =  calculateDistance_angle()
    print(angle, distance, clockwise)
    newX,newY,newZ = CordCalulcation(angle, distance,clockwise)
    print(newX , newY,newZ)
    drone.moveToPositionAsync(newX , newY, newZ  , 3 , vehicle_name='0').join()

testcase1()

    















