import airsim  # type: ignore
import time
import math
import geopy.distance # type: ignore


# Convert Unreal Engine coordinates to GPS
def unreal_to_gps(ue_x, ue_y, ue_z, home_gps):
    """
    Converts Unreal Engine (UE) coordinates to GPS coordinates.
    """
    home_lat, home_lon, home_alt = home_gps.latitude, home_gps.longitude, home_gps.altitude

    # Convert UE X and Y to GPS using geopy
    new_lat_lon = geopy.distance.distance(meters=ue_x).destination((home_lat, home_lon), bearing=0)  # North-South
    # Use the resulting tuple and then apply the Y conversion (East-West)
    new_lat_lon = geopy.distance.distance(meters=ue_y).destination(new_lat_lon, bearing=90)  # East-West

    new_lat = new_lat_lon[0]  # Extract latitude
    new_lon = new_lat_lon[1]  # Extract longitude

    # Convert UE Z to GPS Altitude (UE Z is negative when going up)
    new_alt = home_alt - ue_z  

    return new_lat, new_lon, new_alt


def move_drone_relative(drone_name, x, y, z, speed):
    state = client.getMultirotorState(vehicle_name=drone_name)
    current_position = state.kinematics_estimated.position
    new_position = airsim.Vector3r(current_position.x_val + x, current_position.y_val + y, current_position.z_val + z)    
    client.moveToPositionAsync(new_position.x_val, new_position.y_val, new_position.z_val, speed, vehicle_name=drone_name).join()
    return new_position



# Connect to the AirSim simulator
droneName = "0"
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, droneName)
client.armDisarm(True, droneName)
client.takeoffAsync(vehicle_name=droneName).join()

# Move up to 20 meters altitude
client.moveToZAsync(z=-20, velocity=8, vehicle_name=droneName).join()

# Get home GPS coordinates
gps_home = client.getHomeGeoPoint()
print("Initial GPS Coordinates: ", gps_home)

# Get initial Unreal Engine coordinates
state = client.getMultirotorState(vehicle_name=droneName)
current_position = state.kinematics_estimated.position
print(f"Initial UE Position: {current_position.x_val}, {current_position.y_val}, {current_position.z_val}")

# Move to the right (X-axis) by 10 meters
new_position = move_drone_relative(droneName, 10, 0, 0, 5)

time.sleep(5)  # Allow some time for movement

# Get final Unreal Engine coordinates
state = client.getMultirotorState(vehicle_name=droneName)
current_position = state.kinematics_estimated.position
print(f"Final UE Position: {current_position.x_val}, {current_position.y_val}, {current_position.z_val}")





unreal_coords = (current_position.x_val, current_position.y_val, current_position.z_val)
gps_coords = unreal_to_gps(*unreal_coords, gps_home)
print("Converted GPS Coordinates:", gps_coords)

# Actual GPS from AirSim
actual_gps = client.getMultirotorState(vehicle_name=droneName).gps_location
print("Actual GPS Coordinates from AirSim:", actual_gps)
