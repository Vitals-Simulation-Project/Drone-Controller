
    # def move_drone_relative(drone_name, x, y, z, speed):
    #     state = client.getMultirotorState(vehicle_name=drone_name)
    #     current_position = state.kinematics_estimated.position
    #     new_position = airsim.Vector3r(current_position.x_val + x, current_position.y_val + y, current_position.z_val + z)    
    #     client.moveToPositionAsync(new_position.x_val, new_position.y_val, new_position.z_val, speed, vehicle_name=drone_name).join()
    #     return new_position
    


    # def move_drone_gps_avoid_collision(drone_name, latitude, longitude, altitude, velocity):
    #     client.moveToGPSAsync(latitude, longitude, altitude, velocity, vehicle_name=drone_name)
    #     t_end = time.time() + 30
    #     while (time.time()<t_end):
    #     ##print("im in here")
    #         if (client.getDistanceSensorData(distance_sensor_name='Distance', vehicle_name=drone_name).distance<5 or client.getDistanceSensorData(distance_sensor_name='Distance2', vehicle_name=drone_name).distance<5):
    #             gpsData = client.getGpsData(vehicle_name=drone_name).gnss.geo_point
            
    #             client.moveToGPSAsync(gpsData.latitude,gpsData.latitude,gpsData.altitude+10,3).join()
    #             client.moveToGPSAsync(latitude, longitude, altitude, velocity, vehicle_name=drone_name)
    #             print("moving up")