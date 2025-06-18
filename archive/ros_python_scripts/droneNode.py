#import setup_path # If setup_path gives issue, comment out
import airsim # type: ignore
import numpy as np
import os
import tempfile
import sys
# import pprint
import cv2
import random
import time
import re
import json
import math
from threading import Timer
# from msg import GPSYaw
# from geometry_msgs import Point32

# global variables
LOCAL_IP = "172.21.96.1"
LOCATION_TOPIC = '/DroneLocation'
MIN_VELOCITY = 3
MAX_VELOCITY = 20
RUNTIME = 30
BOID_STEP_TIMER = 1

# Boids
AVOID_FACTOR = 5000 # Separation
CENTERING_FACTOR = 10000 # Cohesion
MATCHING_FACTOR = 1 # Alignment

def globalVarSetup(droneCount, droneName):
    global DRONE_POSITIONS
    DRONE_POSITIONS = [None]*droneCount
    global DRONE_NUMBER
    DRONE_NUMBER = droneCount
    global DRONE_NAME
    DRONE_NAME = droneName


def singleDroneController(droneName, droneCount):
    print("In single controller, Drone name: " + droneName)
    globalVarSetup(droneCount, droneName)

    # Sets client and takes off drone
    client = takeOff(DRONE_NAME)
    client.moveToZAsync(z = -20, velocity = 30, vehicle_name=DRONE_NAME).join()
    rand = random.uniform(-10, 10)
    client.moveByVelocityAsync(rand, rand, 0, duration = 5, vehicle_name=DRONE_NAME)

    # Ros is not a fan of numbers being node names, even if its a string
    # nodeName = "Drone_" + DRONE_NAME
    # rospy.init_node(nodeName, anonymous = True)
    # t = Timer(1, locationSub, args=(DRONE_NAME, client, DRONE_NUMBER)) # every 1.0 seconds
    # t.start()
    # droneController(DRONE_NAME, client, DRONE_NUMBER)


# # Connects subcriber listen to /DroneLocation
# def locationSub(droneName, client, droneCount):
#     rospy.Subscriber(LOCATION_TOPIC, String, updateDroneLocation, (droneCount, client))
#     rospy.spin()

# # Publishes json location data to publishers
# def droneController(droneName, client, droneCount):
#     pub = rospy.Publisher('/DroneLocation', String, latch=True, queue_size=1)
#     updateState(pub, client, droneName)
#     while (not all(d is not None for d in DRONE_POSITIONS)):
#         time.sleep(1)

#     # time.sleep(int(droneName))
#     i = 0
#     while (not rospy.is_shutdown() and i < RUNTIME):
#         updateState(pub, client, droneName)
#         # boids step
#         vector = boidStep(client, int(droneName))
#         client.moveByVelocityAsync(vector[0], vector[1], 0, duration = 5, vehicle_name=droneName)
#         time.sleep(BOID_STEP_TIMER)
#         i+=1

#     print('done')

# def updateState(pub, client, droneName):
#         position = client.getMultirotorState(vehicle_name = droneName)
#         velocity = client.getGpsData(vehicle_name = droneName)

#         DRONE_POSITIONS[int(droneName)] = (droneName, position.gps_location.latitude, position.gps_location.longitude, velocity.gnss.velocity.x_val, velocity.gnss.velocity.y_val) # save locally

#         jsonLocation = json.dumps({"DroneName": droneName, "Longitude": position.gps_location.latitude, "Latitude": position.gps_location.longitude, "vx": velocity.gnss.velocity.x_val, "vy": velocity.gnss.velocity.y_val}, indent=4)
#         pub.publish(jsonLocation) # publish lcoation

# # Gets average x location of the swarm
# def averageXCalculator():
#     averageX = 0
#     for index in range(DRONE_NUMBER):
#         averageX += DRONE_POSITIONS[index][1]
#     averageX = averageX / DRONE_NUMBER
#     return averageX

# # Gets average y location of the swarm
# def averageYCalculator():
#     averageY = 0
#     for index in range(DRONE_NUMBER):
#         averageY += DRONE_POSITIONS[index][2]
#     averageY = averageY / DRONE_NUMBER
#     return averageY

# # Gets average velocity in x direction of swarm
# def averageVXCalculator():
#     averageVX = 0
#     for index in range(DRONE_NUMBER):
#         averageVX += DRONE_POSITIONS[index][3]
#     averageVX = averageVX / DRONE_NUMBER
#     return averageVX

# # Gets average velocity in y direction of swarm
# def averageVYCalculator():
#     averageVY = 0
#     for index in range(DRONE_NUMBER):
#         averageVY += DRONE_POSITIONS[index][4]
#     averageVY = averageVY / DRONE_NUMBER
#     return averageVY


# def cohesion(client, curDroneIndex):

#     averageX = averageXCalculator()
#     averageY = averageYCalculator()
#     boidVX = (averageX - DRONE_POSITIONS[curDroneIndex][1])*CENTERING_FACTOR
#     boidVY = (averageY - DRONE_POSITIONS[curDroneIndex][2])*CENTERING_FACTOR

#     return [boidVX, boidVY]

# def separation(client, curDroneIndex):
#     # Variables used for separation for drones too close
#     close_dx = 0
#     close_dy = 0
#     finalVX = 0
#     finalVY = 0

#     # Make smaller number bigger and big numbers smaller
#     for nearbyDroneIndex in range(DRONE_NUMBER):
#         # Get difference in location from drones
#         xDifference = DRONE_POSITIONS[curDroneIndex][1] - DRONE_POSITIONS[nearbyDroneIndex][1]
#         yDifference = DRONE_POSITIONS[curDroneIndex][2] - DRONE_POSITIONS[nearbyDroneIndex][2]

#         # Check if any nearby drones are too close using difference values
#         if ((nearbyDroneIndex != curDroneIndex) and (abs(xDifference) < 0.00005) and (abs(yDifference) < 0.00005)):
#             close_dx += xDifference
#             close_dy += yDifference

#     finalVX += close_dx*AVOID_FACTOR
#     finalVY += close_dy*AVOID_FACTOR
#     return [finalVX, finalVY]



# def alignment(client, curDroneIndex):
#     # Gets average swarm velocity in x and y
#     averageVX = averageVXCalculator()
#     averageVY = averageVYCalculator()

#     # Added 1 to difference to push drones towards x direction
#     finalVX = (averageVX)*MATCHING_FACTOR
#     finalVY = (averageVY)*MATCHING_FACTOR

#     return [finalVX, finalVY]

# def boidStep(client, curDroneIndex):
#     velocityA = alignment(client, curDroneIndex)
#     velocityC = cohesion(client, curDroneIndex)
#     velocityS = separation(client, curDroneIndex)
#     finalVelocityX = (velocityA[0] + velocityC[0] + velocityS[0])
#     finalVelocityY = (velocityA[1] + velocityC[1] + velocityS[1])

#     speed = math.sqrt(finalVelocityX * finalVelocityX + finalVelocityY * finalVelocityY)
#     if (speed < MIN_VELOCITY):
#         finalVelocityX = (finalVelocityX / speed) * MIN_VELOCITY
#         finalVelocityY = (finalVelocityY / speed) * MIN_VELOCITY
#     elif (speed > MAX_VELOCITY):
#         finalVelocityX = (finalVelocityX / speed) * MAX_VELOCITY
#         finalVelocityY = (finalVelocityY / speed) * MAX_VELOCITY

#     vector = [finalVelocityX, finalVelocityY]

#     print("Drone :", curDroneIndex, "Alignment: ", velocityA, "Cohesion: ", velocityC, "Separation: ", velocityS, "Final V: ", vector)
#     return vector


# # Stores drone position to array, moves drone towards average location
# def updateDroneLocation(data, args):

#     droneCount = args[0]
#     client = args[1]

#     # Gets drone numbers and saves json data
#     testData = str(data.data)
#     droneLocationJson = json.loads(testData)

#     curDroneIndex = int(droneLocationJson['DroneName'])
#     DRONE_POSITIONS[curDroneIndex] = (droneLocationJson['DroneName'], droneLocationJson['Longitude'], droneLocationJson['Latitude'], droneLocationJson['vx'], droneLocationJson['vy'])



# Enables api control, takes off drone, returns the client
def takeOff(droneName):
    client = airsim.MultirotorClient(LOCAL_IP)
    client.confirmConnection()
    client.enableApiControl(True, droneName)
    client.armDisarm(True, droneName)
    client.takeoffAsync(vehicle_name=droneName).join()

    print("Drone " + droneName + " is ready to fly")

    return client

