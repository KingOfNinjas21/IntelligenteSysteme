# Introduction to Intelligent and Autonomus Systems, UIBK, 2017
#contact:senka.krivic@uibk.ac.at
# 

import vrep
import numpy as np
import time
import math
import movementFunctions as move
import rangeSensorFunctions as rangeSen
import bugFunctions as bug
import sys

goalName = "Goal"
LEFT_RAY_NINETY = 603
RIGHT_RAY_NINETY = 82

def main():
    print ('Program started')
    emptybuf = bytearray()

    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 2000, 5)

    removeModel(clientID, "youBot2")

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

        # programable space --------------------------------------------------------------------------------------------

        # initialize sensor and get sensor handles:
        rangeSen.initializeSensor(clientID)
        hokuyo = rangeSen.getSensorHandles(clientID)
        #move.rotateUntilOrientation(clientID, -135.0)

        bug.distB(clientID, hokuyo, goalName)
        #bug.followBoundary(clientID, hokuyo, True)
        #bug.wallOrient(clientID, hokuyo, LEFT_RAY_NINETY, False)

        """
        res, rayHit = move.headTowardsModel(clientID, goalName, hokuyo)

        side = move.wallOrient(clientID, hokuyo, rayHit)
        followBoundary(clientID, hokuyo, side)
        """
        #move.headTowardsModel(clientID, goalName, hokuyo)
        #move.sideway(1.0, clientID, True);

        # Stop simulation ----------------------------------------------------------------------------------------------
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')



# This function represents a bug one algorithm.
def bug1(clientID, sensorHandles):
    # todo head toward goal

    # todo when obstacle hit follow boundary
    startPoint, x = move.getPos(clientID)                       # starting point when following boundary
    minPoint = startPoint                                       # shortest point to target while following boundary
def removeModel(clientID, name):
    res,toRemove=vrep.simxGetObjectHandle(clientID, name, vrep.simx_opmode_blocking)
    vrep.simxRemoveModel(clientID, toRemove, vrep.simx_opmode_oneshot)

if __name__ == "__main__": main()
