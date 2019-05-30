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

        # start init range sensor
        # initialize sensor and get sensor handles:
        rangeSen.initializeSensor(clientID)
        hokuyo = rangeSen.getSensorHandles(clientID)
        # end init range sensor

        # programmable space -------------------------------------------------------------------------------------------

        # implement state machine
        # 1 - init | 2 - detectBlob | 3 - moveToBlob | 4 - grab | 5 - moveBack | 6 - moveToNextGoal |
        # 0 - finish | -1 - finish with error
        state = 1

        # space to store data to share between states
        h_matrix = -1
        path = Queue()

        while state != 0:
            if state == 1:                  # init
                state, explorePath, h_matrix = init_state(youBotCam, clientID)
                path = explorePath

            elif state == 2:                # TODO implement detectBlob
                state=6

            elif state == 3:                # TODO implement moveToBlob
                state=-1

            elif state == 4:                # TODO implement grab
                state=-1

            elif state == 5:                # TODO implement moveBack
                state=-1

            elif state == 6:
                #moveToNextGoal
                if(not path.empty()):
                    finished = ex.distB(clientID, hokuyo, path.get())
                    if(finished):
                        state=6
                    else:
                        state=-1
                else:
                    state=0
                
            elif state == -1:               # finish with error
                print("An error has occurred. Program finished with state -1.")
                state = 0


        # end of programmable space --------------------------------------------------------------------------------------------

        # Stop simulation
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)

        # Close connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')


def init_state(youBotCam, clientID):
    path = ex.initPath()
    #colorDet.get_H_matrix(c.gCX, youBotCam, clientID)
    # init H-Matrix
    return 2, path, 1


if __name__ == "__main__": main()
