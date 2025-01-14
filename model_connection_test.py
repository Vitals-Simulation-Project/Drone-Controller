import airsim # type: ignore
import os
import numpy as np
import cv2
import pprint
import time




# set up client object to access multirotor drone
client = airsim.MultirotorClient()

# connect to AirSim simulator
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)