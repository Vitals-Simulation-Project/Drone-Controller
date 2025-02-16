import airsim # type: ignore
import os
import numpy as np
import cv2
import pprint
import time
from scipy.spatial.transform import Rotation as R # type: ignore


pixel_x, pixel_y = 700, 300  # Change based on your image

drone = "0"
camera_name = "front-" + drone


def move_drone_relative(drone_name, x, y, z, speed):
    state = client.getMultirotorState(vehicle_name=drone_name)
    current_position = state.kinematics_estimated.position
    new_position = airsim.Vector3r(current_position.x_val + x, current_position.y_val + y, current_position.z_val + z)    
    client.moveToPositionAsync(new_position.x_val, new_position.y_val, new_position.z_val, speed, vehicle_name=drone_name).join()
    return new_position


def get_real_world_coordinates(client, pixel_x, pixel_y, camera_name, drone_name):
    # === Get Drone & LiDAR State ===
    drone_state = client.getMultirotorState().kinematics_estimated
    drone_pos = drone_state.position
    drone_orient = drone_state.orientation

    # Convert drone orientation (quaternion) to rotation matrix
    rot = R.from_quat([drone_orient.x_val, drone_orient.y_val, drone_orient.z_val, drone_orient.w_val])
    rotation_matrix = rot.as_matrix()

    # === Get Camera Intrinsics ===
    responses = client.simGetImages([airsim.ImageRequest(camera_name, airsim.ImageType.Scene, False, False)])

    cam_info = client.simGetCameraInfo(camera_name)

    if responses and responses[0].height > 0:
        img_width = responses[0].width
        img_height = responses[0].height
        img_data = responses[0].image_data_uint8
    else:
        print("Error: Could not retrieve image dimensions.")
        img_width, img_height = 640, 480  # Default fallback values
    fov_rad = np.deg2rad(cam_info.fov)

    img = np.frombuffer(img_data, dtype=np.uint8).reshape(img_height, img_width, 3)
    img_copy = img.copy()

    # Compute focal length in pixels
    f_x = (img_width / 2) / np.tan(fov_rad / 2)
    f_y = f_x  # Assuming square pixels
    c_x, c_y = img_width / 2, img_height / 2

    # === Get LiDAR Data ===
    lidar_data = client.getLidarData(vehicle_name=drone_name)
    if len(lidar_data.point_cloud) < 3:
        print("No LiDAR data received.")
        return None, None, None

    # Reshape LiDAR point cloud into Nx3 matrix (x, y, z)
    lidar_points = np.array(lidar_data.point_cloud, dtype=np.float32).reshape(-1, 3)

    # Convert LiDAR points to the drone's local frame
    drone_position = np.array([drone_pos.x_val, drone_pos.y_val, drone_pos.z_val])
    lidar_to_drone = np.dot(rotation_matrix.T, (lidar_points - drone_position).T).T

    # === Project LiDAR Points onto the Camera Image ===
    pixel_coords = []
    depth_values = []

    for point in lidar_to_drone:
        X_c, Y_c, Z_c = point
        if Z_c <= 0:  # Ignore points behind the camera
            continue

        # Project using camera intrinsics
        u = int((X_c * f_x / Z_c) + c_x)
        v = int((Y_c * f_y / Z_c) + c_y)

        # Store valid projections
        if 0 <= u < img_width and 0 <= v < img_height:
            pixel_coords.append((u, v))
            depth_values.append(Z_c)

    # Convert to NumPy arrays for fast searching
    pixel_coords = np.array(pixel_coords)
    depth_values = np.array(depth_values)

    if len(pixel_coords) == 0:
        print("No valid LiDAR projections onto the image.")
        return None, None, None, None

    # === Find the Closest LiDAR Point to the Selected Pixel ===
    distances = np.linalg.norm(pixel_coords - np.array([pixel_x, pixel_y]), axis=1)
    closest_idx = np.argmin(distances)
    Z_actual = depth_values[closest_idx]


    # Draw a circle on the image at the selected pixel (where the calculation was intended)
    cv2.circle(img_copy, (pixel_x, pixel_y), radius=5, color=(0, 255, 0), thickness=-1)  # Green circle

    # Convert Pixel to Camera Coordinates
    X_c = (pixel_x - c_x) * Z_actual / f_x
    Y_c = (pixel_y - c_y) * Z_actual / f_y

    # Convert Camera Coordinates to World Coordinates
    world_coords = np.dot(rotation_matrix, np.array([X_c, Y_c, Z_actual])) + np.array([drone_pos.x_val, drone_pos.y_val, drone_pos.z_val])

    return world_coords[0], world_coords[1], world_coords[2], img_copy  # (X, Y, Z in world frame)



def take_forward_scene_picture(drone_name):
    camera_name = "front-" + drone_name
    print(f"Taking picture from {camera_name}")
    response = client.simGetImage(camera_name=camera_name, image_type=airsim.ImageType.Scene, vehicle_name=drone_name)
    
    filename = os.path.join("images", f"{camera_name}_scene")

    img1d = np.frombuffer(response, dtype=np.uint8)
    img_rgb = cv2.imdecode(img1d, cv2.IMREAD_COLOR)

    cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png




# directory to store pictures
imgDir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'images')

# create directory if it does not exist
if not os.path.exists(imgDir):
    os.makedirs(imgDir)

# set up client object to access multirotor drone
client = airsim.MultirotorClient()

# connect to AirSim simulator
client.confirmConnection()
client.enableApiControl(True, vehicle_name=drone)
client.armDisarm(True, vehicle_name=drone)

# Async methods returns Future. Call join() to wait for task to complete.
client.takeoffAsync(vehicle_name=drone).join()
client.moveToPositionAsync(0, 0, -20, 5).join()


response = client.simGetImage(camera_name=camera_name, image_type=airsim.ImageType.Scene, vehicle_name=drone)

filename = os.path.join("images", "0_scene")

img1d = np.frombuffer(response, dtype=np.uint8)
img_rgb = cv2.imdecode(img1d, cv2.IMREAD_COLOR)

cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png




real_world_x, real_world_y, real_world_z, img = get_real_world_coordinates(client, pixel_x, pixel_y, camera_name, drone)
print(f"Real-world coordinates: ({real_world_x}, {real_world_y}, {real_world_z})")
# print("First GPS coordinates: ", client.getMultirotorState().gps_location)
# Show the image with the circle
cv2.imshow("Image with LiDAR Projection", img)
cv2.waitKey(0)
cv2.destroyAllWindows()




# gps_data = client.getGpsData(vehicle_name=drone)
# gps_al = gps_data.gnss.geo_point.altitude
# gps_lat = gps_data.gnss.geo_point.latitude
# gps_lon = gps_data.gnss.geo_point.longitude
# print(f"GPS coordinates: ({gps_lat}, {gps_lon}, {gps_al})")

# new_x = real_world_x - gps_lat
# new_y = real_world_y - gps_lon
# new_z = real_world_z + gps_al
# print(f"New GPS coordinates: ({new_x}, {new_y}, {new_z})")

# client.moveToGPSAsync(new_x, new_y, new_z, 5).join()

# print("Second GPS coordinates: ", client.getMultirotorState().gps_location)
move_drone_relative(drone, -real_world_x * 11, -real_world_y * 11, 0, 5)

# end connection
client.armDisarm(False, vehicle_name=drone)
client.enableApiControl(False)
