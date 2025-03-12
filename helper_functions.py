import base64
import geopy.distance # type: ignore

def reconstruct_image_from_base64(base64_string, output_path):
    try:
        image_data = base64.b64decode(base64_string)
        with open(output_path, 'wb') as file:
            file.write(image_data)
        print(f"Image successfully reconstructed and saved to {output_path}")
    except Exception as e:
        print(f"An error occurred: {e}")

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