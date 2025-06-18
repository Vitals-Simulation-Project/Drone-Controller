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
client.moveToZAsync(z=-20, velocity=8, vehicle_name=droneName).join()

# go down to 5 meters above ground
while client.getDistanceSensorData(distance_sensor_name='Distance', vehicle_name=droneName).distance > 5:
    # get current position
    drone_state = client.getMultirotorState(vehicle_name=droneName)
    drone_position = drone_state.kinematics_estimated.position
    print("Current altitude: ", drone_position.z_val)
    new_z = drone_position.z_val + 3
    client.moveToZAsync(z=new_z, velocity=5, vehicle_name=droneName).join()
    time.sleep(0.5)