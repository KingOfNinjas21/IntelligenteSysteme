# Introduction to Intelligent and Autonomus Systems, UIBK, 2017
#contact:senka.krivic@uibk.ac.at
#

import vrep
import numpy as np
import cv2
import time
import math
import colorDetection as colorDet
import movementFunctions as move
import aStar
import constants as c
from queue import Queue
import exploreFun as ex
import bugFunctions as bug
import rangeSensorFunctions as rangeSen

def main():
    global clientID

    print ('Program started')
    emptybuf = bytearray()

    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 2000, 5)


    if clientID!=-1:
        print ('Connected to remote API server')

        # Start the simulation:
        vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)

        # start init wheels --------------------------------------------------------------------------------------------

        wheelJoints = np.empty(4, dtype=np.int)
        wheelJoints.fill(-1)  # front left, rear left, rear right, front right
        res, wheelJoints[0] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep.simx_opmode_oneshot_wait)
        res, wheelJoints[1] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_oneshot_wait)
        res, wheelJoints[2] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep.simx_opmode_oneshot_wait)
        res, wheelJoints[3] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_oneshot_wait)

        # end init wheels ----------------------------------------------------------------------------------------------

        # start init camera --------------------------------------------------------------------------------------------

        # change the angle of the camera view (default is pi/4)
        res = vrep.simxSetFloatSignal(clientID, 'rgbd_sensor_scan_angle', math.pi / 2, vrep.simx_opmode_oneshot_wait)

        # turn on camera
        res = vrep.simxSetIntegerSignal(clientID, 'handle_rgb_sensor', 2, vrep.simx_opmode_oneshot_wait);

        # get camera object-handle
        res, youBotCam = vrep.simxGetObjectHandle(clientID, 'rgbSensor', vrep.simx_opmode_oneshot_wait)

        # get first image
        err, resolution, image = vrep.simxGetVisionSensorImage(clientID, youBotCam, 0, vrep.simx_opmode_streaming)
        time.sleep(1)

        # end init camera ----------------------------------------------------------------------------------------------

        # start init range sensor ----------------------------------------------------------------------------------------------
        # initialize sensor and get sensor handles:
        rangeSen.initializeSensor(clientID)
        hokuyo = rangeSen.getSensorHandles(clientID)

        # end init range sensor ----------------------------------------------------------------------------------------------

        # programmable space -------------------------------------------------------------------------------------------

        # implement state machine
        # 1 - init | 2 - detect blob | 3 - move to blob | 4 - grab | 5 - follow next explore path | 6 - align to blob | 7 - follow next basket path
        # 8 - drop block | 0 - finish | -1 - finish with error

        state = 1

        # space to store data to share between states
        h_matrix = -1
        explorePaths = Queue()
        basketPaths = Queue()
        blobsList = []
        visitedBlobsList = []
        orientations = Queue()
        posBeforeMoveToBlob = move.getPos(clientID)[0][:-1]

        while state != 0:
            if state == 1:      # init
                # init path, H, and next state
                state, explorePaths, basketPaths, orientations, h_matrix = ex.init_state(youBotCam, clientID)

            elif state == 2:    # detect blob
                # find all blobs that are 360 degrees around youBot
                state, blobsList = ex.findBlobs(clientID, youBotCam, h_matrix, blobsList, visitedBlobsList)

            elif state == 3:    # move to blob
                # get to next blob state
                posBeforeMoveToBlob = move.getPos(clientID)[0][:-1] # store current position for moving back later
                state, blobsList, visitedBlobsList = ex.getToNextBlob(clientID, blobsList, visitedBlobsList)

            elif state == 4:    # grab
                state, blobsList = ex.grabBlob(clientID, h_matrix, youBotCam)

            elif state == 5:    # follow next explore path
                state = ex.followExplorePath(clientID, hokuyo, explorePaths, orientations)

            elif state == 6:    # align to blob
                state = ex.alignToBlob(clientID)

            elif state == 7:    # follow next basket path
                state = ex.followBasketPath(clientID, hokuyo, basketPaths)

            elif state == 8:    # drop blob
                state = ex.dropBlob(clientID)

            elif state == -1:               # finish with error
                print("Current state: fail state!")
                print("An error has occurred. Program finished with state -1.")
                state = 0
        print("End of blob grabing shit")


        #ex.moveArm(clientID, -90, 20,70,0,0)
        #ex.moveArm(clientID, -90, 90,0,0,0)
        #ex.getAngle(clientID)
        #ex.moveArm(clientID, 0, 0,0,0,0)
       # ex.moveArm(clientID, 180/math.pi*ex.getAngle(clientID), 95,40,35,0)
        # end of programmable space --------------------------------------------------------------------------------------------

        # Stop simulation
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)

        # Close connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')


if __name__ == "__main__": main()
