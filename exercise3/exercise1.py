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

goalName = "Goal"

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

        distB(clientID, hokuyo)
        #followBoundary(clientID, hokuyo, True)


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


def followBoundary(clientID, sensorHandles, rightSide):
    # 603 und 82
    if rightSide:
        rayHit = 603
    else:
        rayHit = 82
    rotateAngle = 1.0
    rangeToWallOld = 0.5
    minRange = rangeToWallOld-0.1
    maxRange = rangeToWallOld+0.1
    counter = 0
    rangeData = rangeSen.getSensorData(clientID, sensorHandles)
    minDistToTarget = -1.0
    oldDis = rangeData[rayHit][3]
    newDis = rangeData[rayHit][3]

    while True:
        move.startMoving(clientID)
        while not move.detectCorner(clientID,sensorHandles, rayHit, (minRange+maxRange)/2.0): #not abs(oldDis - newDis)>1
            oldDis = newDis
            rangeData = rangeSen.getSensorData(clientID, sensorHandles)

            for i in range(move.FRONT_SEN_START,move.FRONT_SEN_END):          # check for a pool of front sensors if there is an obstacle
                if rangeData[i][3] <= 0.5:
                    move.wallOrient(clientID, sensorHandles, rayHit, True)
                    break

            move.startMoving(clientID)
            if counter % 5 == 0:

                rangeToWallNew = rangeData[rayHit][3]
                print(rangeToWallNew)
                if rangeToWallNew<minRange:
                    move.setWheelVelocity(clientID, 0)
                    move.sideway(minRange -rangeToWallNew ,clientID,rightSide)
                    move.startMoving(clientID)
                    move.wallOrient(clientID, sensorHandles, rayHit, False)
                    #move.rotate(rotateAngle, clientID, True)
                elif rangeToWallNew>maxRange:
                    move.setWheelVelocity(clientID, 0)
                    move.sideway(rangeToWallNew - maxRange ,clientID, not rightSide)
                    move.startMoving(clientID)
                    move.wallOrient(clientID, sensorHandles, rayHit, False)
                    #move.rotate(rotateAngle, clientID, False)

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

            newDis = rangeData[rayHit][3]



        print("drive around corner")

        goAroundCorner(clientID, sensorHandles, rightSide, rayHit)

        #TODO: check leaving conditions here
        freespace = move.calcFreeSpace(clientID, sensorHandles)
        leavingCondition, minDistToTarget = move.calcLeavingConditin(minDistToTarget, freespace, clientID)

        if leavingCondition:
            print("leaving cause of leaving condition")
            return
        #elif(True):
            # condition target is visible holds



def goAroundCorner(clientID, sensorHandles, rightSide, rayHit):
    print("start going around corner")
    move.forward(0.8, clientID)
    move.rotate(90, clientID, not rightSide)
    move.forward(0.8, clientID)

    if(normalBorder(clientID, sensorHandles, rightSide)):
        print("just a corner")

    else:
        print("U-Turn")
        while True:
            move.forward(0.2, clientID)
            rangeData = rangeSen.getSensorData(clientID, sensorHandles)
            countRays = 0
            for i in range(rayHit - 80, rayHit + 80):
                if rangeData[i][3] < 0.8:
                    countRays += 1
            if countRays < 5:
                break
        move.rotate(90, clientID, not rightSide)
        move.forward(1.0, clientID)
        move.wallOrient(clientID, sensorHandles, rayHit, False)
    print("end going around corner")

def normalBorder(clientID, sensorHandles, rayHit):
    rangeData = rangeSen.getSensorData(clientID, sensorHandles)
    for i in range(rayHit-15, rayHit+15):
        for j in range(i+5, i+30):
            if(abs(rangeData[i][3] - rangeData[j][3]) > 0.2):
                return False
    #for i in range(rayHit+10, rayHit + 50):
    #    if(abs(rangeData[i][3]-rangeData[rayHit][3])<0.5):
    #        return True

    return True

def distB(clientID, sensorHandles):
    isGoal, hitRay = move.headTowardsModel(clientID, goalName, sensorHandles)
    while (isGoal):
        isGoal, hitRay = move.headTowardsModel(clientID, goalName, sensorHandles)

        rangeData = rangeSen.getSensorData(clientID, sensorHandles)
        if(move.isChairInFront(rangeData)):
            move.driveAroundChair(clientID)
        else:
            isRight = move.wallOrient(clientID, sensorHandles, hitRay, False)
            followBoundary(clientID, sensorHandles, isRight)
        print("is goal:", isGoal)

def removeModel(clientID, name):
    res,toRemove=vrep.simxGetObjectHandle(clientID, name, vrep.simx_opmode_blocking)
    vrep.simxRemoveModel(clientID, toRemove, vrep.simx_opmode_oneshot)

if __name__ == "__main__": main()
