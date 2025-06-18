import airsim # type: ignore
import os
import numpy as np
import cv2
import pprint
import time
import geopy.distance # type: ignore
import base64



drone = "0"



# set up client object to access multirotor drone
client = airsim.MultirotorClient()

# connect to AirSim simulator
client.confirmConnection()
client.enableApiControl(True, vehicle_name=drone)
client.armDisarm(True, vehicle_name=drone)

client.takeoffAsync(vehicle_name=drone).join()


client.simSetSubWindowCamera(0, "front-1", "1")

time.sleep(30)

# end connection
client.enableApiControl(False)
