# Introduction to Intelligent and Autonomus Systems, UIBK, 2017
#contact:senka.krivic@uibk.ac.at
#

import vrep
import numpy as np
import cv2
import array
import time
import math
import movementFunctions as move
import rangeSensorFunctions as rangeSen
import bugFunctions as bug
import sys
from PIL import Image

goalName = "Goal"
LEFT_RAY_NINETY = 603
RIGHT_RAY_NINETY = 82

def main():
    print ('Program started')
    emptybuf = bytearray()

    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 2000, 5)


    if clientID!=-1:
        print ('Connected to remote API server')


        # Start the simulation:
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait)

        wheelJoints = np.empty(4, dtype=np.int)
        wheelJoints.fill(-1)  # front left, rear left, rear right, front right
        res, wheelJoints[0] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep.simx_opmode_oneshot_wait)
        res, wheelJoints[1] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_oneshot_wait)
        res, wheelJoints[2] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep.simx_opmode_oneshot_wait)
        res, wheelJoints[3] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_oneshot_wait)

        # initialize sensor and get sensor handles:
        rangeSen.initializeSensor(clientID)
        hokuyo = rangeSen.getSensorHandles(clientID)

        # programable space --------------------------------------------------------------------------------------------

        # change the angle of the camera view (default is pi/4)
        res = vrep.simxSetFloatSignal(clientID, 'rgbd_sensor_scan_angle', math.pi / 2, vrep.simx_opmode_oneshot_wait)

        # turn on camera
        res = vrep.simxSetIntegerSignal(clientID, 'handle_rgb_sensor', 2, vrep.simx_opmode_oneshot_wait);

        # get camera object-handle
        res, youBotCam = vrep.simxGetObjectHandle(clientID, 'rgbSensor', vrep.simx_opmode_oneshot_wait)

        # get first image
        err, resolution, image = vrep.simxGetVisionSensorImage(clientID, youBotCam, 0, vrep.simx_opmode_streaming)
        time.sleep(1)

        while (vrep.simxGetConnectionId(clientID) != -1):
            # get further images from vision sensor
            err, res, image = vrep.simxGetVisionSensorImage(clientID, youBotCam, 0, vrep.simx_opmode_buffer)
            if err == vrep.simx_return_ok:
                # transformation to byte array
                image_byte_array = array.array('b', image)

                # transformation to opencv2 image
                #image_buffer = Image.frombuffer("RGB", (res[0], res[1]), image_byte_array, "raw", "RGB", 0, 1)
                # if python 3, you may instead need to do:
                image_buffer = Image.frombuffer("RGB", (res[0],res[1]), np.asarray(image_byte_array), "raw", "RGB", 0, 1)
                # or
                # image_buffer = Image.frombuffer("RGB", (res[0],res[1]), bytes(image_byte_array), "raw", "RGB", 0, 1) , use
                img = np.asarray(image_buffer)
                # Convert RGB to BGR
                img = img[:, :, ::-1].copy()
                cv2.imshow("img from vrep", img)
                cv2.waitKey(0)
                #cv2.imwrite("imageFromVREP.png", image_buffer, cv2.IMWRITE_PNG_COMPRESSION)

        # Stop simulation ----------------------------------------------------------------------------------------------
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')

if __name__ == "__main__": main()
