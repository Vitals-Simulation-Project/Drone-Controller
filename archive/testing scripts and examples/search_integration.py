import airsim  # type: ignore
import time
import os
import numpy as np
import cv2
import math
from stopwatch import Stopwatch # type: ignore

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
confirm_target_speed = 6           # Speed (m/s)

drone = "0"                        # Drone

imgDir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'images')

if not os.path.exists(imgDir):
    os.makedirs(imgDir)


def waypoint_search(client, center_x, center_y, side_length, altitude, speed):
    global running

    half_side = side_length / 2  
    
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
        client.rotateToYawAsync(get_yaw_angle_to_target(client,center_x,center_y)).join() #rotate to center
        client.hoverAsync()
        time.sleep(3)
        
        if (create_mask() == True): # take pictures
            return
    
def create_mask():
    
    #img = client.simGetImage(camera_name="front_center", image_type= airsim.ImageType.Infrared, vehicle_name=drone)
    #depth = client.simGetImages([airsim.ImageRequest("front_center", airsim.ImageType.DepthPerspective, True, False)])
    img = client.simGetImage(camera_name="front-0", image_type= airsim.ImageType.Infrared, vehicle_name=drone)
    depth = client.simGetImages([airsim.ImageRequest("front-0", airsim.ImageType.DepthPerspective, True, False)])
    print(depth[0].height, depth[0].width)
    
    img1d = np.frombuffer(img, dtype=np.uint8)  ## this section is creating the black and white mask to outline anything in the infared
    img_rgb = cv2.imdecode(img1d, cv2.IMREAD_COLOR)
    hsv = cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)
    #print(depth[0].height, depth[0].width)

    lower = np.array([0,0,0], dtype = "uint8")
    upper = np.array([192,192,192], dtype = "uint8") #can change upper lower if u want to change what gets out lined

    mask = cv2.inRange(hsv, lower, upper) 
    
    # here 36864
    cv2.imshow("Mask",mask) ## comment these back in when testing it will show u what it is seeing
    cv2.waitKey(0)
    try:     
        horizontal = np.argwhere(mask)[3][1] # change the first [] to decide how many pixels it needs to see to move
        calculateDistance_angle(mask,depth[0])
        return True
    except IndexError: #exceptions are for when there is nothing
        return False
    except ValueError: #this is for a math domain error u can delete this if u ever find why 
        bool = create_mask()
        return bool

    
def confirm_target_search(client, center_x, center_y, side_length, altitude, speed):
    global running
    
    half_side = side_length / 2  

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

        client.rotateToYawAsync(get_yaw_angle_to_target(client,center_x,center_y)).join()  
        client.hoverAsync()
        time.sleep(3)
        
        #take photo here / VLM integration
        print("taking photo")


def get_yaw_angle_to_target(client, x,y): # this gets the drone to face the center of the search
    drone_state = client.getMultirotorState()
    drone_position = drone_state.kinematics_estimated.position

    dx = x - drone_position.x_val
    dy = y - drone_position.y_val

    yaw = math.atan2(dy, dx)
    yaw_deg = math.degrees(yaw) % 360

    return yaw_deg

def create_waypoint(): # this is for test case / can be deleted
    state = client.getMultirotorState()
    currentposition = state.kinematics_estimated.position
    
    client.moveToPositionAsync(currentposition.x_val,currentposition.y_val , currentposition.z_val-15 ,10, vehicle_name= drone).join()
    client.moveToPositionAsync(currentposition.x_val+200,currentposition.y_val-12 , currentposition.z_val-15 ,20, vehicle_name= drone).join() 
    currentposition = state.kinematics_estimated.position
    return currentposition

def MoveToGPSAsyncCollsion(self, latitude, longitude, altitude, velocity, timeout_sec = 3e+38,  vehicle_name = ''): # not used needs to be updated the movement we use LATER
    self.moveToGPSAsync(latitude,longitude,altitude,velocity,vehicle_name= vehicle_name)
    t_end = time.time() + 30
    while (time.time()<t_end): ## need a solution to this its just a bandaid rn 
        if (self.getDistanceSensorData(distance_sensor_name='Distance').distance<5 or self.getDistanceSensorData(distance_sensor_name='Distance2').distance<5):
            gpsData = self.getGpsData().gnss.geo_point
        
            self.moveToGPSAsync(gpsData.latitude,gpsData.latitude,gpsData.altitude+10,3).join()
            self.moveToGPSAsync(latitude,longitude,altitude,velocity,vehicle_name= vehicle_name)
            #print("moving up")
    self.hoverAsync

def calculateDistance_angle(mask,depthPerspective):
    client.hoverAsync()
    
    depth_img_in_meters = airsim.list_to_2d_float_array(depthPerspective.image_data_float, depthPerspective.width, depthPerspective.height) 
    
    depth_img_in_meters = depth_img_in_meters.reshape(depthPerspective.height, depthPerspective.width, 1) # height is 144 , width is 256
    

    height= client.getDistanceSensorData(distance_sensor_name='Distance').distance # gets altitude / hight from ground of drone
  
    perpixel = 90/256  # FOV / the horizontal image size
  
    horizontal = np.argwhere(mask)[3][1] # finds where the white pixels in the mask are
    
    if(horizontal >= (256/2)):
        #print("right")
        clockwise = True
    elif(horizontal<(256/2)): 
        #print("left")
        clockwise = False   
    vert = np.argwhere(mask)[3][0]
    distance = depth_img_in_meters[vert][horizontal][0] #plugs in where the white is into the depth map to get the distance
    

    calculations = math.sqrt(distance**2 - height**2)  # the big P triangle theorm

    
    horizontalangle = perpixel * horizontal
    print(horizontalangle, calculations, clockwise)
    CordCalulcation(horizontalangle, calculations, clockwise)

def to_eularian_angles(q): # takes eularian's and give pitch roll yaw
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

    state = client.getMultirotorState()
    q = state.kinematics_estimated.orientation
    roll , pitch, yaw = to_eularian_angles(q)
    if (clockwise == True): # takes into account where the drone is in the fov and finds angle
        angle = abs ( 45 - angle)
        yaw = yaw + angle
    elif(clockwise == False):
        angle = abs(angle -45)
        yaw = yaw - angle
  
    client.rotateToYawAsync(yaw).join()  #rotate to the target
    time.sleep(1) #sleep to give it time
  
    
   
    stopwatch = Stopwatch(2)
    stopwatch.start()
    client.moveByVelocityBodyFrameAsync(5,0,0,distance/5)
    
    while(stopwatch.duration<(distance /5)):

        if (client.getDistanceSensorData(distance_sensor_name='Distance').distance<5 or client.getDistanceSensorData(distance_sensor_name='Distance2').distance<5):
            state = client.getMultirotorState()
            currentposition = state.kinematics_estimated.position

           
            stopwatch.stop()
            print("moving up ")
            client.moveToPositionAsync(currentposition.x_val,currentposition.y_val , currentposition.z_val-5 ,3, vehicle_name= drone).join()
            #client.moveByVelocityZAsync(3,3,3,3).join()
            print("done moving up ")

            newdistance = distance - (5 * (stopwatch.duration))
            stopwatch.start()
            client.moveByVelocityBodyFrameAsync(5,0,0,newdistance/5) # travels to the target
            #print("moving up")
        
    #client.moveByVelocityBodyFrameAsync(5,0,0,distance/5).join() # travels to the target
    #if i need to collision avodiance
    stopwatch.stop()
    stopwatch.reset()
    print("out of movement")
    #collision call
    #new distance = distance - (5* time end - time start)
    # continue 
   
    state = client.getMultirotorState()
    currentposition = state.kinematics_estimated.position
    #print(currentposition)
    time.sleep(1) # gives time to stop

    if( distance > 80):
        print("to far doing a waypoint")
        waypoint_search(client,currentposition.x_val, currentposition.y_val, confirm_target_side_length, currentposition.z_val, confirm_target_speed)
    else:
        confirm_target_search(client, currentposition.x_val, currentposition.y_val, confirm_target_side_length, currentposition.z_val, confirm_target_speed)
    
    def MoveToPositionAsyncCollsion(self, X, Y, altitude, velocity, drone_name, timeout_sec = 3e+38 ): # not used needs to be updated the movement we use LATER
        self.moveToPositionAsync(X,Y,altitude,velocity,drivetrain=airsim.DrivetrainType.ForwardOnly,yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0), vechicle_name= drone_name )
        t_end = time.time() + 30
        while (time.time()<t_end): ## need a solution to this its just a bandaid rn 
            if (self.getDistanceSensorData(distance_sensor_name='Distance').distance<5 or self.getDistanceSensorData(distance_sensor_name='Distance2').distance<5):
                state = self.getMultirotorState()
                currentposition = state.kinematics_estimated.positiont
            
                self.moveToPositionAsync(currentposition.x_val,currentposition.y_val,altitude-10,velocity,drivetrain=airsim.DrivetrainType.ForwardOnly,yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0), vechicle_name= drone_name )
                self.moveToPositionAsync(X,Y,currentposition.z_val,velocity,drivetrain=airsim.DrivetrainType.ForwardOnly,yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0), vechicle_name= drone_name )
                #print("moving up")
        self.hoverAsync

try:
    state = client.getMultirotorState()
    currentposition = state.kinematics_estimated.position
    create_waypoint() 
    
    time.sleep(3)
    waypoint_search(client, 230, -12, waypoint_side_length, waypoint_altitude, waypoint_speed) # Change to the waypoint given
 

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





    