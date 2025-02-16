import airsim # type: ignore
# import Constants.configDrones as configDrones
# import ImageProcessing.getInfo as getInfo
import time
import math


def unreal_to_gps(unreal_x, unreal_y, unreal_z, home_gps):
    earth_radius = 6378137.0  # Earth's radius in meters

    # Extract home latitude, longitude, altitude
    lat_home = math.radians(home_gps.latitude)
    lon_home = math.radians(home_gps.longitude)
    alt_home = home_gps.altitude

    # Convert Unreal coordinates (ENU system)
    d_lat = (unreal_y / earth_radius) * (180 / math.pi)
    d_lon = (unreal_x / (earth_radius * math.cos(lat_home))) * (180 / math.pi)
    d_alt = -unreal_z  # Unreal's Z is negative for altitude

    # New GPS coordinates
    new_lat = home_gps.latitude + d_lat
    new_lon = home_gps.longitude + d_lon
    new_alt = alt_home + d_alt

    return new_lat, new_lon, new_alt





# Connect to the AirSim simulator
droneName = "0"
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, droneName)
client.armDisarm(True, droneName)
client.takeoffAsync(vehicle_name=droneName).join()

client.moveToZAsync(z=-20, velocity=8, vehicle_name = droneName).join()


gps_home = client.getHomeGeoPoint()
print("Initial coordinates: ", gps_home)
state = client.getMultirotorState(vehicle_name=droneName)
current_position = state.kinematics_estimated.position
print(f"Initial position: {current_position.x_val}, {current_position.y_val}, {current_position.z_val}")


# move to the right 10 meters
client.moveToPositionAsync(10, 0, -20, 5, vehicle_name=droneName).join()


time.sleep(5)


state = client.getMultirotorState(vehicle_name=droneName)
current_position = state.kinematics_estimated.position
print(f"Final position: {current_position.x_val}, {current_position.y_val}, {current_position.z_val}")


unreal_coords = (current_position.x_val, current_position.y_val, current_position.z_val)
gps_coords = unreal_to_gps(*unreal_coords, gps_home)
# add to home coordinates
print("Converted GPS:", gps_coords)


print("Actual GPS:", client.getMultirotorState(vehicle_name=droneName).gps_location)




