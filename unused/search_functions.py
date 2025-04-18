import airsim  # type: ignore
import time
import os
import numpy as np
import cv2
import math


# Constants
CONFIRM_TARGET_ALTITUDE = -10      # Fixed altitude (negative for AirSim)
CONFIRM_TARGET_SIDE_LENGTH = 10    # Square size
CONFIRM_TARGET_SPEED = 6           # Speed (m/s)



def waypoint_search(client, drone_name, center_x, center_y, side_length, altitude, speed):
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
        print("Going to waypoint: ", x, y)
        client.moveToPositionAsync(
            x, y, altitude, speed,
            drivetrain=airsim.DrivetrainType.ForwardOnly,
            yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0),
            vehicle_name=drone_name  
        ).join()
        client.rotateToYawAsync(get_yaw_angle_to_target(client, drone_name, center_x, center_y), vehicle_name=drone_name).join() #rotate to center
        client.hoverAsync(vehicle_name=drone_name)
        time.sleep(3)
        
        if (create_mask(client, drone_name) == True): # take pictures
            print("Target found")
            return
        else:
            print("Target not found")
            continue
    
def create_mask(client, drone_name):
    #camera_name = "front-" + drone_name
    camera_name = "front_center"
    img = client.simGetImage(camera_name=camera_name, image_type=airsim.ImageType.Infrared, vehicle_name=drone_name)
    depth = client.simGetImages([airsim.ImageRequest(camera_name, airsim.ImageType.DepthPerspective, True, False)], vehicle_name=drone_name)
    print("Took infrared and depth images")
    img1d = np.frombuffer(img, dtype=np.uint8)  # this section is creating the black and white mask to outline anything in the infared
    img_rgb = cv2.imdecode(img1d, cv2.IMREAD_COLOR)
    hsv = cv2.cvtColor(img_rgb,cv2.COLOR_BGR2HSV)

    lower = np.array([0,0,0], dtype = "uint8")
    upper = np.array([192,192,192], dtype = "uint8") #can change upper lower if u want to change what gets out lined

    mask = cv2.inRange(hsv, lower, upper)   
    cv2.imshow("Mask",mask) ## comment these back in when testing it will show u what it is seeing
    cv2.waitKey(0)
    try:     
        horizontal = np.argwhere(mask)[3][1] # change the first [] to decide how many pixels it needs to see to move
        calculateDistance_angle(client, drone_name, mask, depth[0])
        return True
    except IndexError: #exceptions are for when there is nothing
        return False
    except ValueError: #this is for a math domain error u can delete this if u ever find why 
        bool = create_mask(client, drone_name)
        return bool

    
def confirm_target_search(client, drone_name, center_x, center_y, side_length, altitude, speed):
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
            yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0),
            vehicle_name=drone_name  
        ).join()

        client.rotateToYawAsync(get_yaw_angle_to_target(client, drone_name, center_x, center_y), vehicle_name=drone_name).join()  
        client.hoverAsync(vehicle_name=drone_name)
        time.sleep(3)
        
        # TODO: take photo here / VLM integration 
        # base64_picture = take_forward_picture(drone_name, airsim.ImageType.Scene)
        # image_queue.put(Image(drone_name, "Scene", base64_picture, current_target.name))



def get_yaw_angle_to_target(client, drone_name, x, y): # this gets the drone to face the center of the search
    drone_state = client.getMultirotorState(vehicle_name=drone_name)
    drone_position = drone_state.kinematics_estimated.position

    dx = x - drone_position.x_val
    dy = y - drone_position.y_val

    yaw = math.atan2(dy, dx)
    yaw_deg = math.degrees(yaw) % 360

    return yaw_deg


def calculateDistance_angle(client, drone_name, mask, depthPerspective):
    client.hoverAsync(vehicle_name=drone_name)
    
    depth_img_in_meters = airsim.list_to_2d_float_array(depthPerspective.image_data_float, depthPerspective.width, depthPerspective.height) 
    
    depth_img_in_meters = depth_img_in_meters.reshape(depthPerspective.height, depthPerspective.width, 1) # height is 144 , width is 256
    

    height = client.getDistanceSensorData(distance_sensor_name='Distance', vehicle_name=drone_name).distance # gets altitude / hight from ground of drone
  
    perpixel = 90/256  # FOV / the horizontal image size
  
    horizontal = np.argwhere(mask)[3][1] # finds where the white pixels in the mask are
    
    if(horizontal >= (256 / 2)):
        #print("right")
        clockwise = True
    elif(horizontal<(256 / 2)): 
        #print("left")
        clockwise = False   
    vert = np.argwhere(mask)[3][0]
    distance = depth_img_in_meters[vert][horizontal][0] #plugs in where the white is into the depth map to get the distance


    calculations = math.sqrt(distance**2 - height**2)  # the big P triangle theorm

    
    horizontalangle = perpixel * horizontal
    #print(horizontalangle, calculations, clockwise)
    cord_calculation(client, drone_name, horizontalangle, calculations, clockwise)

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

def cord_calculation(client, drone_name, angle, distance, clockwise):

    state = client.getMultirotorState(vehicle_name=drone_name)
    q = state.kinematics_estimated.orientation
    roll , pitch, yaw = to_eularian_angles(q)
    if (clockwise == True): # takes into account where the drone is in the fov and finds angle
        angle = abs ( 45 - angle)
        yaw = yaw + angle
    elif(clockwise == False):
        angle = abs(angle -45)
        yaw = yaw - angle
  
    print("rotating to yaw: ", yaw)
    client.rotateToYawAsync(yaw, vehicle_name=drone_name).join()  #rotate to the target
    time.sleep(1) #sleep to give it time
  
    
    client.moveByVelocityBodyFrameAsync(5, 0, 0, distance / 5, vehicle_name=drone_name).join() # travels to the target
   
    state = client.getMultirotorState(vehicle_name=drone_name)
    currentposition = state.kinematics_estimated.position
    #print(currentposition)
    time.sleep(1) # gives time to stop

   
    confirm_target_search(client, drone_name, currentposition.x_val, currentposition.y_val, CONFIRM_TARGET_SIDE_LENGTH, currentposition.z_val, CONFIRM_TARGET_SPEED)
    

