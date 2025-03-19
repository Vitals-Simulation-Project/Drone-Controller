from pydantic import BaseModel # type: ignore


# Waypoint class to store the name, location, and priority of each waypoint
# using a priority queue to prioritize waypoints
# location is a list of x, y, z Unreal Engine coordinates
# priority is 1 for user-assigned waypoints, 2 for dropped waypoints that must be revisited, and 3 for regular waypoints
class Waypoint:
    def __init__(self, name, x, y, z, priority):
        self.name = name
        self.x = x
        self.y = y
        self.z = z
        self.priority = priority

    def __lt__(self, other):
        return self.priority > other.priority 


# Image class to store the drone ID, image type, and image data
# image type can be "airsim.ImageType.Scene" or "airsim.ImageType.Infrared"
# image is stored as base64 encoded string
# drone_id is the id of the drone that took the image
# waypoint is the name of the waypoint where the image was taken
class Image:
    def __init__(self, drone_id, image_type, image, waypoint_name):
        self.drone_id = drone_id
        self.image_type = image_type
        self.image = image
        self.waypoint_name = waypoint_name
    



class VLMOutput(BaseModel):
    #assigned_target_dictionary: dict # drone maps to waypoint
    drone_id: int
    image_result: str # can be "heat signature detected", "no heat signature detected", "target confirmed", "target not confirmed"
    target_location: tuple # location of the target
