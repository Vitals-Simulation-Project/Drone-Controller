import airsim #type: ignore
import pprint

# connect to the AirSim simulator
def sendCoordinates():
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    gps_data = client.getGpsData()
    s = pprint.pformat(gps_data)
    print("gps_data: %s" % s)
    return gps_data