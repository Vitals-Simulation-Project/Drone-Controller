import torch
import numpy as np
import cv2
from PIL import Image
from ImageProcessing import getInfo
from ImageProcessing import getInfoWolf
import os
import rospy
from airsim_ros_pkgs.srv import requestGPU
import Constants.ros as ros
import Constants.configDrones as configDrones
from airsim_ros_pkgs.msg import updateMap
from airsim_ros_pkgs.msg import GPS
import time
import RosPublishHelper.MapHandlerPublishHelper as mapHandlerPublishHelper

# Environmental Variables
GPU_SERVICE = ros.GPU_SERVICE

FINAL_TARGET_POSITION = ros.FINAL_TARGET_POSITION
NEW_GPS_PREDICTION = ros.NEW_GPS_PREDICTION
UPDATE_DRONE_POSITION =  ros.UPDATE_DRONE_POSITION

def runYolov5(responses, dataDir_pass, dataDir_fail, vehicleName, confidanceMin, gps, updateMapPublisher):
    global GPU_SERVICE
    global counter

    responseIndex = 0

    # get response object with input image
    height, width, chunks = getInfo.getHeightWidthArr(responses, responseIndex)
    chunkMaxConfidence = 0.0
    combinedChunksValidDetection = False
    finalMaxConfidenceGPS = [None, None]
    chunksPassedConfidence = False
    bestIndex = -1
    for index, sceneRGB2 in enumerate(chunks):

        #print("post-shape: ", sceneRGB2.shape)
        sceneRGB1 = np.copy(sceneRGB2)

        # j=0
        # while os.path.exists('/home/testuser/AirSim/PythonClient/multirotor/Drone-Search-and-Rescue-SD/DroneSwarm/testSceneRGB/' + str(j)+ 'testPreCorrupt' + '.png'):
        #     j+=1

        # cv2.imwrite('/home/testuser/AirSim/PythonClient/multirotor/Drone-Search-and-Rescue-SD/DroneSwarm/testSceneRGB/' + str(j) + 'testPreCorrupt' + '.png', sceneRGB1)
        

        # original image unedited

        responseString= responses[int(responseIndex)].image_data_uint8

        # Requests gpu service and sends over response string, gets response object
        gpuServiceTime = time.time()
        rospy.wait_for_service(GPU_SERVICE)
        response = rospy.ServiceProxy(GPU_SERVICE, requestGPU)
        array_bytes = sceneRGB2.tobytes()
        responseObject = response(str(array_bytes.decode('latin-1')), height, width)
        gpuLen = time.time() - gpuServiceTime
        # print("gpuLen:       " + str(gpuLen) + "         999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999")
        # print("Yolo still running")
        # print("gpuLen:       " + str(gpuLen) + "         999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999999")

        # Set variables from response object
        success = responseObject.success
        xmin = int(responseObject.xMin)
        ymin = int(responseObject.yMin)
        xmax = int(responseObject.xMax)
        ymax = int(responseObject.yMax)
        confidence = responseObject.confidence

        maxConfidence = 0
        maxConfidenceGPS = [None, None]
        detection = 0
        maxConfidenceDetection = 0

        #ToDo: just pass loop index adn drone name
        # or in final build just overwite
        j=1
        while os.path.exists(dataDir_pass + "/" + ('%s' % j)+"w"+vehicleName+"camImg.jpg"):
            j+=1

        #ToDo: just pass loop index adn drone name
        # or in final build just overwite
        k=1
        while os.path.exists(dataDir_fail + "/" + ('%s' % k)+"w"+vehicleName+"camImg.jpg"):
            k+=1

        validDetection = False
        passedConfidence = False
        # TODO: CHANGE WITH SUCCESS BOOL FROM ROS
        if(success):
            validDetection = True
            confidence = responseObject.confidence
            # if confidence is high enough use for GPS estimation
            if(confidence >= confidanceMin):
                passedConfidence=True
                
                print("High Confidence Detection!!! Index: ", index)

                start_point = (xmin, ymin)
                end_point = (xmax, ymax)
                cv2.rectangle(sceneRGB2, start_point, end_point, (0, 255, 0), 2)
                # save new image only with highest confidence detection
                cv2.imwrite(dataDir_pass + "/" + ('%s' % j)+"w"+vehicleName+"camImg.jpg", sceneRGB1)
                cv2.imwrite(dataDir_pass + "/" + ('%s' % j)+"w"+vehicleName+"bbImg.jpg", sceneRGB2)

                #print("------------------------------------------------------------------------------------------------------------------")
                # use bb dimensions/location for GPS estimation
                alt, lat, lon = getInfoWolf.getWolfGPSEstimate(responses, xmin, ymin, xmax, ymax, gps)

                # Got a detection
                # wolf gps visualized
                mapHandlerPublishHelper.updateWolfDronePrediction(wolfMapPublisher=updateMapPublisher, droneName=vehicleName, imageNumber=j, currentGPS=gps, targetLat=lat, targetLon=lon)
                # getInfo.getDetectionMap((gps[1], gps[2]), (lat, lon), j, vehicleName)

                #print("\tWOLF ESTIMATE: "+str(alt)+" alt, " + str(lat) + " lat, " + str(lon) + " lon")
                maxConfidenceGPS[1]=lat
                maxConfidenceGPS[0]=lon
                #print("------------------------------------------------------------------------------------------------------------------")

                # write corresponding text file
                with open(dataDir_pass + "/" + ('%s' % j)+"w"+vehicleName+"GPSEstimate.txt", 'w') as f:
                    # f.write(str(resultsPandas))
                    f.write("\n\tMax Confidence: "+str(confidence))
                    f.write("\n\tMax Confidence Estimate:"+ str(lat) + " lat, " + str(lon) + " lon")
                    f.close()

            else:
                passedConfidence=False
                print("Low Confidence Detection!!! Index: ", index)

                start_point = (xmin, ymin)
                end_point = (xmax, ymax)
                cv2.rectangle(sceneRGB2, start_point, end_point, (0, 255, 0), 2)
                # save new image only with highest confidence detection
                cv2.imwrite(dataDir_fail + "/" + ('%s' % k)+"w"+vehicleName+"camImg.jpg", sceneRGB1)
                cv2.imwrite(dataDir_fail + "/" + ('%s' % k)+"w"+vehicleName+"bbImg.jpg", sceneRGB2)

                #print("------------------------------------------------------------------------------------------------------------------")
                # use bb dimensions/location for GPS estimation
                alt, lat, lon = getInfoWolf.getWolfGPSEstimate(responses, xmin, ymin, xmax, ymax, gps)

                # wolf gps visualized
                mapHandlerPublishHelper.updateWolfDroneFailPrediction(wolfMapPublisher=updateMapPublisher, droneName=vehicleName, imageNumber=k, currentGPS=gps, targetLat=lat, targetLon=lon)
            
                #print("\tWOLF ESTIMATE: "+str(alt)+" alt, " + str(lat) + " lat, " + str(lon) + " lon")
                maxConfidenceGPS[1]=lat
                maxConfidenceGPS[0]=lon
                #print("------------------------------------------------------------------------------------------------------------------")
                # ToDO consider showing loc confidance detections

                # write corresponding text file
                with open(dataDir_fail + "/" + ('%s' % k)+"w"+vehicleName+"GPSEstimate.txt", 'w') as f:
                    # f.write(str(resultsPandas))
                    f.write("\n\tMax Confidence: "+str(confidence))
                    f.write("\n\tMax Confidence Estimate:"+ str(lat) + " lat, " + str(lon) + " lon")
                    f.close()

                # Did not get a detection but updating position of the wolf
                # wolf gps visualized

        else:
            mapHandlerPublishHelper.updateWolfDronePosition(wolfMapPublisher=updateMapPublisher, droneName=vehicleName, currentGPS=gps)
        if (confidence > chunkMaxConfidence):
            chunkMaxConfidence = confidence
            combinedChunksValidDetection = validDetection
            finalMaxConfidenceGPS = maxConfidenceGPS
            chunksPassedConfidence = passedConfidence
            bestIndex = index

    return finalMaxConfidenceGPS, combinedChunksValidDetection, chunksPassedConfidence