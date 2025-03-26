import airsim  # type: ignore
import time
import math


# Connect to the AirSim simulator
droneName = "0"
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, droneName)
client.armDisarm(True, droneName)
client.takeoffAsync(vehicle_name=droneName).join()

# Move up to 20 meters altitude
client.moveToZAsync(z=-5, velocity=8, vehicle_name=droneName).join()

for i in range(10):
    client.rotateToYawAsync(180, vehicle_name=droneName).join()
    time.sleep(1) 
    client.rotateToYawAsync(0, vehicle_name=droneName).join()
    time.sleep(1)