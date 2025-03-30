import airsim  # type: ignore
import multiprocessing as mp
import random
import time
import os
import numpy as np
import cv2
import local_config
import base64
import heapq
import asyncio
import math


from classes import Image
from helper_functions import unreal_to_gps
#import search_functions


# Constants
MIN_ALTITUDE = 10
MIN_FORWARD_DISTANCE = 15
VELOCITY = 15

WAYPOINT_ALTITUDE = -15            # Fixed altitude (negative for AirSim)
WAYPOINT_SIDE_LENGTH = 10          # Square size
WAYPOINT_SPEED = 8                 # Speed (m/s)

CONFIRM_TARGET_ALTITUDE = -10      # Fixed altitude (negative for AirSim)
CONFIRM_TARGET_SIDE_LENGTH = 10    # Square size
CONFIRM_TARGET_SPEED = 6           # Speed (m/s)










def singleDroneController(drone_name, current_target_dictionary, status_dictionary, target_found, searched_areas_dictionary, image_queue, waypoint_queue, current_position_dictionary):
    """ Drone process that listens for movement commands and sends status updates. """
    

    # Enables api control, takes off drone, returns the client
    def takeOff(drone_name):
        client = airsim.MultirotorClient(local_config.LOCAL_IP)
        client.confirmConnection()
        client.enableApiControl(True, drone_name)
        client.armDisarm(True, drone_name)
        client.takeoffAsync(vehicle_name=drone_name).join()

        # print("Drone " + drone_name + " is ready to fly")
        status_dictionary[drone_name] = "IDLE"

        return client





    def take_forward_picture(drone_name, image_type):
        camera_name = "front-" + drone_name
        print(f"Taking picture from {camera_name}, type of {image_type}")
        response = client.simGetImage(camera_name=camera_name, image_type=image_type, vehicle_name=drone_name)
        
        filename = os.path.join("images", f"{camera_name}_scene_{image_type}")

        img1d = np.frombuffer(response, dtype=np.uint8)


        if image_type == airsim.ImageType.Infrared:
            img_gray = cv2.imdecode(img1d, cv2.IMREAD_GRAYSCALE)
            cv2.imwrite(os.path.normpath(filename + '.png'), img_gray) # write to png on disk
            _, buffer = cv2.imencode('.png', img_gray)  # Encode the image as PNG
            base64_image = base64.b64encode(buffer).decode('utf-8')  # Convert to base64
        else:
            img_rgb = cv2.imdecode(img1d, cv2.IMREAD_COLOR)
            cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png on disk
            _, buffer = cv2.imencode('.png', img_rgb)  # Encode the image as PNG
            base64_image = base64.b64encode(buffer).decode('utf-8')  # Convert to base64

    
        return base64_image  
    


    
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
        camera_name = "front-" + drone_name
        #camera_name = "front_center"
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
            horizontal = np.argwhere(mask)[4][1] # change the first [] to decide how many pixels it needs to see to move
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
                x, y, -MIN_ALTITUDE, speed,
                drivetrain=airsim.DrivetrainType.ForwardOnly,
                yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0),
                vehicle_name=drone_name  
            ).join()

            client.rotateToYawAsync(get_yaw_angle_to_target(client, drone_name, center_x, center_y), vehicle_name=drone_name).join()  
            client.hoverAsync(vehicle_name=drone_name)
            time.sleep(3)

            # go down to 10 meters above ground
            while client.getDistanceSensorData(distance_sensor_name='Distance', vehicle_name=drone_name).distance > 10:
                # get current position
                drone_state = client.getMultirotorState(vehicle_name=drone_name)
                drone_position = drone_state.kinematics_estimated.position
                print("Current altitude: ", drone_position.z_val)
                new_z = drone_position.z_val + 3
                client.moveToZAsync(z=new_z, velocity=5, vehicle_name=drone_name).join()
                time.sleep(0.5)

            time.sleep(1)
            print("Taking picture for vlm")
            # take photo for vlm to analyze
            base64_picture = take_forward_picture(drone_name, airsim.ImageType.Scene)
            image_queue.put(Image(drone_name, "Scene", base64_picture, current_target.name))



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
    
        horizontal = np.argwhere(mask)[4][1] # finds where the white pixels in the mask are
        
        if(horizontal >= (256 / 2)):
            #print("right")
            clockwise = True
        elif(horizontal<(256 / 2)): 
            #print("left")
            clockwise = False   
        vert = np.argwhere(mask)[4][0]
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
    
        
        client.moveByVelocityBodyFrameAsync(7, 0, 0, distance / 7, vehicle_name=drone_name).join() # travels to the target
        state = client.getMultirotorState(vehicle_name=drone_name)
        currentposition = state.kinematics_estimated.position
        #print(currentposition)
        time.sleep(1) # gives time to stop

        if( distance > 80):
            #print("to far doing a waypoint")
            waypoint_search(client,currentposition.x_val, currentposition.y_val, CONFIRM_TARGET_SIDE_LENGTH, currentposition.z_val, CONFIRM_TARGET_SPEED)
        else:
            confirm_target_search(client, drone_name, currentposition.x_val, currentposition.y_val, CONFIRM_TARGET_SIDE_LENGTH, currentposition.z_val, CONFIRM_TARGET_SPEED)
    




    # Initialize AirSim client and take off
    client = takeOff(drone_name)


    while not target_found.value:
        current_target = current_target_dictionary[drone_name]

        if current_target is not None:
            waypoint_name = current_target.name
            waypoint_lat, waypoint_lon, waypoint_alt = unreal_to_gps(current_target.x, current_target.y, current_target.z, client.getHomeGeoPoint())
            waypoint_x = current_target.x
            waypoint_y = current_target.y
            waypoint_z = current_target.z

            print(f"Drone {drone_name} is moving to {waypoint_name}")
            print("Going to GPS coordinates: ", waypoint_lat, waypoint_lon, waypoint_alt)
            print("Going to Unreal coordinates: ", waypoint_x, waypoint_y, waypoint_z)

            print("Rotating to face waypoint using yaw: ", get_yaw_angle_to_target(client, drone_name, waypoint_x, waypoint_y))
            client.rotateToYawAsync(get_yaw_angle_to_target(client, drone_name, waypoint_x, waypoint_y), vehicle_name=drone_name).join()

            move_future = client.moveToGPSAsync(waypoint_lat, waypoint_lon, waypoint_alt + MIN_ALTITUDE, VELOCITY, vehicle_name=drone_name)
            status_dictionary[drone_name] = "MOVING"

            while True:
                # if current_target_dictionary[drone_name] != original_target: (TODO: Fix and test this)
                #     print(f"Drone {drone_name} received a new target while moving to {waypoint_name}")
                #     # interrupt movement with hover
                #     client.hoverAsync().join()

                #     # push the original target back to the queue to be revisited later
                #     heapq.heappush(waypoint_queue, original_target)
                #     break

                drone_state = client.getMultirotorState(vehicle_name=drone_name)
                position = drone_state.kinematics_estimated.position
                current_x, current_y, current_z = position.x_val, position.y_val, position.z_val


                #print("Recorded height: ", client.getDistanceSensorData(distance_sensor_name='Distance', vehicle_name=drone_name).distance)
                if (client.getDistanceSensorData(distance_sensor_name='Distance', vehicle_name=drone_name).distance < MIN_ALTITUDE or client.getDistanceSensorData(distance_sensor_name='Distance2', vehicle_name=drone_name).distance < MIN_FORWARD_DISTANCE):
                    #print("Height below threshold: ", client.getDistanceSensorData(distance_sensor_name='Distance', vehicle_name=drone_name).distance)
                    #print("Or forward distance below threshold: ", client.getDistanceSensorData(distance_sensor_name='Distance2', vehicle_name=drone_name).distance < MIN_ALTITUDE)
                    #gpsData = drone_state.gps_location
                    #client.moveToGPSAsync(gpsData.latitude, gpsData.longitude, gpsData.altitude+5, VELOCITY / 2, vehicle_name=drone_name).join()
                    # move z up
                    client.hoverAsync().join()
                    #print("current height was: " + str(current_z))
                    client.moveToZAsync(current_z - (MIN_ALTITUDE / 2), VELOCITY / 2, vehicle_name=drone_name).join()
                    #print("moving up to ", current_z - (MIN_ALTITUDE / 2))
                    client.rotateToYawAsync(get_yaw_angle_to_target(client, drone_name, waypoint_x, waypoint_y), vehicle_name=drone_name).join()

                    move_future = client.moveToGPSAsync(waypoint_lat, waypoint_lon, waypoint_alt + MIN_ALTITUDE, VELOCITY, vehicle_name=drone_name)

                    #print("continuing to: " + str(waypoint_lat) + " " + str(waypoint_lon) + " " + str(waypoint_alt - MIN_ALTITUDE))


                # Check if the drone is close enough to the target (within a small threshold)
                x_distance = abs(current_x - waypoint_x)
                y_distance = (current_y - waypoint_y)
                #print("Distance to target: ", (x_distance**2 + y_distance**2)**0.5)
                if x_distance < 10 and y_distance < 5:
                    move_future.join()
                    print(f"Drone {drone_name} reached the target")
                    break
                else:
                    move_future = client.moveToGPSAsync(waypoint_lat, waypoint_lon, waypoint_alt + MIN_ALTITUDE, VELOCITY, vehicle_name=drone_name)

                time.sleep(1)
            status_dictionary[drone_name] = "SEARCHING"
            print(f"Drone {drone_name} arrived and is searching {waypoint_name}")

            # hover for 5 seconds
            client.hoverAsync(vehicle_name=drone_name).join()
            time.sleep(5)

            print("Calling search function")
            drone_state = client.getMultirotorState(vehicle_name=drone_name)
            position = drone_state.kinematics_estimated.position
            current_x, current_y, current_z = position.x_val, position.y_val, position.z_val
            print("Current position: ", current_x, current_y, current_z)

            #waypoint_search(client, drone_name, current_x, current_y, WAYPOINT_SIDE_LENGTH, current_z, WAYPOINT_SPEED)
            #print("Search function finished")
            # Take a picture
            # base64_picture = take_forward_picture(drone_name, airsim.ImageType.Scene)

            # image_queue.put(Image(drone_name, "Scene", base64_picture, current_target.name))



            time.sleep(5)

            current_target_dictionary[drone_name] = None
            print(f"Drone {drone_name} finished searching {waypoint_name}")
            searched_areas_dictionary[waypoint_name] = (waypoint_lat, waypoint_lon, waypoint_alt)
            status_dictionary[drone_name] = "IDLE"


        else:
            print(f"Drone {drone_name} is waiting for commands.")
            current_position = client.getMultirotorState(vehicle_name=drone_name).kinematics_estimated.position
            current_position_dictionary[drone_name] = (current_position.x_val, current_position.y_val, current_position.z_val)
            #print("Current position: ", current_position_dictionary[drone_name])
            #status_dictionary[drone_name] = "IDLE"
            time.sleep(10)




if __name__ == "__main__":
    print("This script is not meant to be run directly. Please run mission_control.py instead.")
    exit(1)