# Introduction to Intelligent and Autonomus Systems, UIBK, 2017
#contact:senka.krivic@uibk.ac.at
# 

import vrep
import numpy as np
import time
import math
import movementFunctions as move
import rangeSensorFunctions as rangeSen
import sys

def main():
    print ('Program started')
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 2000, 5)

    removeModel(clientID, "youBot2")

    if clientID!=-1:
        print ('Connected to remote API server')
    
        emptyBuff = bytearray()
    
        # Start the simulation:
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait)
    
        # initialize robot
        # Retrieve wheel joint handles:
        wheelJoints=np.empty(4, dtype=np.int); wheelJoints.fill(-1) #front left, rear left, rear right, front right
        res,wheelJoints[0]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fl',vrep.simx_opmode_oneshot_wait)
        res,wheelJoints[1]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rl',vrep.simx_opmode_oneshot_wait)
        res,wheelJoints[2]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fr',vrep.simx_opmode_oneshot_wait)
        res,wheelJoints[3]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rr',vrep.simx_opmode_oneshot_wait)


        # initialize sensor and get sensor handles:
        rangeSen.initializeSensor(clientID)
        hokuyo = rangeSen.getSensorHandles(clientID)
        res, rayHit = move.headTowardsModel(clientID, "Goal", hokuyo)
        #move.wallOrient(clientID,hokuyo, rayHit)

        #reason, rayHit = move.headTowardsModel(clientID, "Goal", hokuyo)
        side = move.wallOrient(clientID, hokuyo, rayHit)
        #print("Wall orient returned: ", side)
        followBoundary(clientID, hokuyo, side)
        move.headTowardsModel(clientID, "Goal", hokuyo)
        #move.rotate(90, clientID, False)


        # Stop simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')


def followBoundary(clientID, sensorHandles, rightSide):
    # 603 und 82
    if rightSide:
        rayHit = 603
    else:
        rayHit = 82
    rotateAngle = 1.0
    rangeToWallOld = rangeSen.getSensorData(clientID, sensorHandles)[rayHit][3]
    minRange = rangeToWallOld-0.1
    maxRange = rangeToWallOld+0.1
    counter = 0

    minDistToTarget = -1.0


    while True:
        while not move.detectCorner(clientID, sensorHandles, rayHit, rightSide):
            rangeData = rangeSen.getSensorData(clientID, sensorHandles)
            if rangeData[307][3]<0.5 or rangeData[378][3] < 0.5:
                move.wallOrient(clientID, sensorHandles, 307)



            move.startMoving(clientID)
            if counter % 5 == 0:

                rangeToWallNew = rangeData[rayHit][3]
                print(rangeToWallNew)
                if rangeToWallNew<minRange:
                    move.rotate(rotateAngle, clientID, True)
                elif rangeToWallNew>maxRange:
                    move.rotate(rotateAngle, clientID, False)

                #else:
                #    move.wallOrient(clientID, sensorHandles, 603)
                rangeToWallOld = rangeToWallNew

            counter+=1

            # calc stuff for leaving condition
            freespace = move.calcFreeSpace(clientID, sensorHandles)
            leavingCondition, minDistToTarget = move.calcLeavingConditin(minDistToTarget, freespace, clientID)

            if leavingCondition:
                print("leaving cause of leaving condition")
                return


        print("drive around corner")

        move.forward(0.8, clientID)
        move.rotate(90, clientID, not rightSide)
        move.forward(0.8, clientID)
        while True:
            move.forward(0.2, clientID)
            rangeData = rangeSen.getSensorData(clientID, sensorHandles)
            countRays = 0
            for i in range(rayHit-80, rayHit+80):
                if rangeData[i][3] < 0.8:
                    countRays+=1
            if countRays<5:
                break
        move.rotate(90, clientID, not rightSide)
        move.forward(1.0, clientID)
        move.wallOrient(clientID, sensorHandles, rayHit)


def detectClearPath(clientID):
    # todo: realize clear path
	
	return False


def removeModel(clientID, name):
    res,toRemove=vrep.simxGetObjectHandle(clientID, name, vrep.simx_opmode_blocking)
    vrep.simxRemoveModel(clientID, toRemove, vrep.simx_opmode_oneshot)

if __name__ == "__main__": main()
