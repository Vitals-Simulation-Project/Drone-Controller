import airsim # type: ignore
import tempfile
import os
import pprint

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()

# add new vehicle
vehicle_name = "Drone2"
pose = airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(0, 0, 0))

client.simAddVehicle(vehicle_name, "simpleflight", pose)
client.enableApiControl(True, vehicle_name)
client.armDisarm(True, vehicle_name)
client.takeoffAsync(10.0, vehicle_name)

