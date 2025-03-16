import airsim  # type: ignore
import time
import os
import numpy as np
import cv2
import math

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

client.armDisarm(True)
client.takeoffAsync().join()

waypoint_altitude = -15            # Fixed altitude (negative for AirSim)
waypoint_side_length = 10          # Square size
waypoint_speed = 8                 # Speed (m/s)

confirm_target_altitude = -10      # Fixed altitude (negative for AirSim)
confirm_target_side_length = 10    # Square size
confirm_target_speed = 8           # Speed (m/s)

drone = "0"                        # Drone

imgDir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'images')

if not os.path.exists(imgDir):
    os.makedirs(imgDir)

def take_forward_picture(drone_name, image_type):
    #camera_name = "front-" + drone_name
    camera_name = "front_center"
    print(f"Taking picture from {camera_name}")
    response = client.simGetImage(camera_name=camera_name, image_type=image_type, vehicle_name=drone_name)
    
    filename = os.path.join("images", f"{camera_name}_scene_{image_type}")

    img1d = np.frombuffer(response, dtype=np.uint8)
    img_rgb = cv2.imdecode(img1d, cv2.IMREAD_COLOR)

    cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png
    return response

def waypoint_search(client, center_x, center_y, side_length, altitude, speed):
    global running

    half_side = side_length / 2  
    state = client.getMultirotorState()
    q = state.kinematics_estimated.orientation
   

    # Waypoints for target to be centered
    square_corners = [
        (center_x - half_side, center_y - half_side),  # Bottom-left
        (center_x + half_side, center_y - half_side),  # Bottom-right
        (center_x + half_side, center_y + half_side),  # Top-right
        (center_x - half_side, center_y + half_side),  # Top-left
    ]

    for x, y in square_corners: 
        client.moveToPositionAsync(
            x, y, altitude, speed,
            drivetrain=airsim.DrivetrainType.ForwardOnly,
            yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0)  
        ).join()
        roll , pitch, yaw = to_eularian_angles(q)
        curyaw=yaw
        client.rotateToYawAsync(curyaw+135).join()
        client.hoverAsync()
        time.sleep(5)
        
        if (create_mask() == True):
            return
   # mask = create_mask()
    # try:     
    #     horizontal = np.argwhere(mask)[0][1] ## if its not working this 0 is a 1 # need error handling here if there is nothing
    #     print("going into calc")
    #     calculateDistance_angle( mask)
    #     return
    # except IndexError:
    #     print("continue")
    # except ValueError:
    #     #create_mask()
    #     print("fuck yall")
    
def create_mask():
    infared_image = take_forward_picture(drone, airsim.ImageType.Infrared)

    img = infared_image
    
    img1d = np.frombuffer(img, dtype=np.uint8) ## breaking here rn 
    img_rgb = cv2.imdecode(img1d, cv2.IMREAD_COLOR)
    hsv = cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)
    #print("im here 5")

    ##img.resize(256,144) ## width , height ## this is breaking after the changes/ cant do it / might need this when integrating everyones code

    lower = np.array([0,0,0], dtype = "uint8")
    upper = np.array([192,192,192], dtype = "uint8")

    mask = cv2.inRange(hsv, lower, upper)   
    #print("im here 6")
    cv2.imshow("Mask",mask)
    #cv2.imshow("img",img1d)
    cv2.waitKey(0)
    try:     
        horizontal = np.argwhere(mask)[0][1] ## if its not working this 0 is a 1 # need error handling here if there is nothing
        print("going into calc")
        calculateDistance_angle(mask)
        return True
    except IndexError:
        print("continue")
        return False
    except ValueError:
        bool = create_mask()
        return bool

    #return mask
    
def confirm_target_search(client, center_x, center_y, side_length, altitude, speed):
    global running

    half_side = side_length / 2  
    state = client.getMultirotorState()
    q = state.kinematics_estimated.orientation
    roll , pitch, yaw = to_eularian_angles(q)
    curyaw=yaw

    # Waypoints for target to be centered
    square_corners = [
        (center_x - half_side, center_y - half_side),  # Bottom-left
        (center_x + half_side, center_y - half_side),  # Bottom-right
        (center_x + half_side, center_y + half_side),  # Top-right
        (center_x - half_side, center_y + half_side),  # Top-left
    ]

    for x, y in square_corners: 
        client.moveToPositionAsync(
            x, y, altitude, speed,
            drivetrain=airsim.DrivetrainType.ForwardOnly,
            yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0)  
        ).join()

        # Delay for stabilization
        time.sleep(3)
        client.rotateToYawAsync(curyaw+135).join()

        take_forward_picture(drone, airsim.ImageType.Scene)

def create_waypoint():
    state = client.getMultirotorState()
    currentposition = state.kinematics_estimated.position
    
    client.moveToPositionAsync(currentposition.x_val,currentposition.y_val , currentposition.z_val-15 ,10, vehicle_name= drone).join()
    client.moveToPositionAsync(currentposition.x_val+200,currentposition.y_val-12 , currentposition.z_val-15 ,20, vehicle_name= drone).join() 
    currentposition = state.kinematics_estimated.position
    return currentposition

def MoveToGPSAsyncCollsion(self, latitude, longitude, altitude, velocity, timeout_sec = 3e+38,  vehicle_name = ''):
    self.moveToGPSAsync(latitude,longitude,altitude,velocity,vehicle_name= vehicle_name)
    t_end = time.time() + 30
    while (time.time()<t_end): ## need a solution to this its just a bandaid rn 
        if (self.getDistanceSensorData(distance_sensor_name='Distance').distance<5 or self.getDistanceSensorData(distance_sensor_name='Distance2').distance<5):
            gpsData = self.getGpsData().gnss.geo_point
        
            self.moveToGPSAsync(gpsData.latitude,gpsData.latitude,gpsData.altitude+10,3).join()
            self.moveToGPSAsync(latitude,longitude,altitude,velocity,vehicle_name= vehicle_name)
            #print("moving up")
    self.hoverAsync

def calculateDistance_angle(mask):
    client.hoverAsync()
    #print("im here 1")
    responses = client.simGetImages([airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False),airsim.ImageRequest("front_center", airsim.ImageType.Infrared,False,False)])
    #TEST_response= client.simGetImage(camera_name="front_center", image_type= airsim.ImageType.Infrared, vehicle_name='0') ## throws a fit if i do them both in one call
    depthPerspective = responses[0]

    #depth = responses[0]
    #print("im here 2")
    
    depth_img_in_meters = airsim.list_to_2d_float_array(depthPerspective.image_data_float, depthPerspective.width, depthPerspective.height) 
    #print("im here 3")
    depth_img_in_meters = depth_img_in_meters.reshape(depthPerspective.height, depthPerspective.width, 1)
    #print("height")
    #print( depthPerspective.height)
    #print("width" )
    #print(depthPerspective.width)
    depth_8bit_lerped = np.interp(depth_img_in_meters, (0, 100), (0, 255))

    #print(responses[0].width) #256
    #print("width^^^^^")
  
    #img = responses[1]
    #img = infared
    #print("im here 3")
    # img = infared
  
    # img1d = np.frombuffer(img, dtype=np.uint8) ## breaking here rn 
    # img_rgb = cv2.imdecode(img1d, cv2.IMREAD_COLOR)
    # hsv = cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)
    # print("im here 5")

    # ##img.resize(256,144) ## width , height ## this is breaking after the changes/ cant do it / might need this when integrating everyones code

    # lower = np.array([0,0,0], dtype = "uint8")
    # upper = np.array([192,192,192], dtype = "uint8")

    # mask = cv2.inRange(hsv, lower, upper)
    # #print("im here 6")
    # cv2.imshow("Mask",mask)
    # #cv2.imshow("img",img1d)
    # cv2.waitKey(0)
    # horizontal = np.argwhere(mask)[0][1] ## if its not working this 0 is a 1 # need error handling here if there is nothing
    ##mask.resize(256,288)
    
    ##print("im here 7")

    height= client.getDistanceSensorData(distance_sensor_name='Distance').distance
    ##print("im here 3")
    perpixel = 90/256 ## might  be wrong there is a lot of things to check but in theroy if everything is right it works #half of the big angle # was 256 before david :( 
  
    horizontal = np.argwhere(mask)[0][1] ## if its not working this 0 is a 1 # need error handling here if there is nothing
    
    if(horizontal >= (256/2)): # was 256 before david :(
        print("right")
        clockwise = True
    elif(horizontal<(256/2)): # was 256 before david :(
        print("left")
        clockwise = False   
    vert = np.argwhere(mask)[0][0]
    distance = depth_img_in_meters[vert][horizontal][0] ## could be back wards

    print(depth_img_in_meters[vert][horizontal][0])
    print("depth 0")
    print(depth_img_in_meters[vert][horizontal][1])
    print("depth 1")
    #print(horizontal)
    #print("horizontal ^^^")
    #print(vert)
    #print("vert ^^^^")
    ##print(height)
    #try:
    calculations = math.sqrt(distance**2 - height**2) ## need a catch for this i think 
    #except ValueError:
       # print("math mo fucking domain")
        #return
    horizontalangle = perpixel * horizontal
    #time.sleep(30)
    print(horizontalangle, calculations, clockwise)
    CordCalulcation(horizontalangle, calculations, clockwise)
    ##return horizontalangle , calculations , clockwise
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
    #bearing = client. get its angle from north
    
    state = client.getMultirotorState()
    q = state.kinematics_estimated.orientation
    roll , pitch, yaw = to_eularian_angles(q)
    print(yaw)
    print("yaw^^^")
    if (clockwise == True):
        yaw = yaw + angle
    elif(clockwise == False):
        yaw = yaw - angle
    
    
    client.rotateToYawAsync(200).join() ## correct angle is 200
    time.sleep(3)
    # if (yaw > 180 ):
    #     temp = yaw - 180
    #     yaw = -abs(180-temp)
    #     #yaw = yaw - 90
    #     #yaw = -yaw
    # elif (yaw < -180):
    #     temp = yaw + 180
    #     yaw = abs(180 - temp)
    #     #yaw = yaw + 90
    #     #yaw = abs(yaw)
    currentposition = state.kinematics_estimated.position
    print(yaw)
    # if(yaw >0  and yaw <= 90 ):
    #     print("quad 1")
    #     newY= currentposition.y_val + math.sin(math.radians(angle)) * distance
    #     newX= currentposition.x_val + (math.cos(math.radians(angle)) * distance )
    # elif(yaw >90  and yaw <= 180):
    #     print("quad 2")
    #     newY= currentposition.y_val +math.sin(math.radians(angle)) * distance
    #     newX= currentposition.x_val - (math.cos(math.radians(angle)) * distance )
    # elif((yaw >-180  and yaw <= -90) or (yaw > 180 and yaw <= 270)):
    #     print("quad 3")
    #     newY= currentposition.y_val -math.sin(math.radians(angle)) * distance
    #     newX= currentposition.x_val - (math.cos(math.radians(angle)) * distance )
    # elif((yaw >-90  and yaw <= 0) or (yaw >270 and yaw <= 360)):
    #     print("quad 4")
    #     newY= currentposition.y_val -math.sin(math.radians(angle)) * distance
    #     newX= currentposition.x_val + (math.cos(math.radians(angle)) * distance )
    newY= currentposition.y_val + (math.sin(math.radians(yaw)) * distance)
    newX= currentposition.x_val + (math.cos(math.radians(yaw)) * distance )
    
    currentposition = state.kinematics_estimated.position
    print (currentposition)
    print(newX,newY)
    print("we got all the way down here!!!!!")
    confirm_target_search(client, newX, newY, confirm_target_side_length, currentposition.z_val, confirm_target_speed)
    
    #return newX,newY, currentposition.z_val

try:
    create_waypoint() 
    
    time.sleep(3)
    waypoint_search(client, 230, -12, waypoint_side_length, waypoint_altitude, waypoint_speed) # Change to the waypoint given
    # state = client.getMultirotorState()
    # currentposition = state.kinematics_estimated.position
    # client.moveToPositionAsync(currentposition.x_val,currentposition.y_val , currentposition.z_val-15 ,10, vehicle_name= drone).join()
    # client.moveToPositionAsync(99,-8.8 , currentposition.z_val-15 ,20, vehicle_name= drone).join() 

finally:
    # Land the drone
    print("Landing")
    client.landAsync().join()

    # Check landed state and disarm
    landed_state = client.getMultirotorState().landed_state
    if landed_state == airsim.LandedState.Landed:
        client.armDisarm(False)
        print("Drone successfully disarmed.")
    else:
        print("Drone is not in a landed state. Skipping disarm.")

    # Disable API control
    client.enableApiControl(False)
    print("Flight operation completed.")