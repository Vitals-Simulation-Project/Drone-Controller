import airsim

# set up client object to access multirotor drone
client = airsim.MultirotorClient()

# connect to AirSim simulator
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Async methods returns Future. Call join() to wait for task to complete.
client.takeoffAsync().join()
client.moveToZAsync(100, 5).join()

client.moveToPositionAsync(-1000, -1, 100, 5).join()

client.enableApiControl(False)