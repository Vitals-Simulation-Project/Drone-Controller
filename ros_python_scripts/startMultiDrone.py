#!/usr/bin/env python
#import airsim
#import os
#import numpy as np
#import cv2
# import print
import time
from droneNode import singleDroneController
from ctypes import Structure, c_int
from multiprocessing.sharedctypes import Array

# multipel drones
import multiprocessing as mp
# mp.set_start_method('spawn')

# Airsim multiple drone Contol with Multiprocessing Demo
print('start multiDrone')
if __name__ == '__main__': # Only runs if this is main processes
    mp.set_start_method('fork') # windows specific. Change based on OS.
    droneCount = mp.cpu_count() - 5

    for x in range(4): # str(x) = the vechical_name of the drone
        droneName = str(x)
        print(droneName)
        mp.Process(target=singleDroneController, args=(droneName,5)).start()
