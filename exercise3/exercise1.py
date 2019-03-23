# Introduction to Intelligent and Autonomus Systems, UIBK, 2017
#contact:senka.krivic@uibk.ac.at
# 

import vrep
import numpy as np
import time
import math
import movementFunctions as move
import rangeSensorFunctions as range

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

        '''
        move.printPos(clientID)

        # initialize sensor
        range.initializeSensor(clientID)

        # Get sensor handle:
        hokuyo = range.getSensorHandles(clientID)

        # get range sensor data as list of x,y,z,distance
        rangeData = range.getSensorData(clientID, hokuyo[1])

        print rangeData
        '''
        '''
        #move.forward(3, 0.5, clientID)
        res, base = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_oneshot_wait)
        base_pos = vrep.simxGetObjectPosition(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
        base_orient = vrep.simxGetObjectOrientation(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
        print(base_pos)

        res, base = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_oneshot_wait)
        base_pos = vrep.simxGetObjectPosition(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
        print(base_pos)
        '''

        #move.forward(2.26, clientID)

        headTowardsModel(clientID, "conferenceChair")

        # Stop simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')




def headTowardsModel(clientID, modelName):
    res, objHandle = vrep.simxGetObjectHandle(clientID, modelName, vrep.simx_opmode_oneshot_wait)


    targetPosition = vrep.simxGetObjectPosition(clientID, objHandle, -1, vrep.simx_opmode_oneshot_wait)
    xTarget = targetPosition[1][0]
    yTarget = targetPosition[1][1]
    print ("{}: x= {}, y= {}" .format(modelName,xTarget,yTarget))
    pos, ori = move.getPos(clientID)
    '''
    angle = calcAngleToTarget(clientID, pos[0], pos[1], xTarget, yTarget) % 360
    print("Angle: ", angle)
    if angle>0:
        move.rotate(angle, clientID, True)
    else:
        move.rotate(angle, clientID, False)
    
    '''
    print(pos[0], " ", pos[1])
    dist = calcDistanceToTarget(pos[0], pos[1], xTarget, yTarget)
    move.forward(dist, clientID)

# calculates distance between 2 x,y coordinates
def calcDistanceToTarget(xStart, yStart, xEnd, yEnd):
    distanceToTarget = math.sqrt((xEnd-xStart)*(xEnd-xStart)+(yEnd-yStart)*(yEnd-yStart))
    print(distanceToTarget)
    return distanceToTarget

def calcAngleToTarget(clientID, xStart, yStart, xEnd, yEnd):
    GK = float(yEnd-yStart)
    AK = float(xEnd-xStart)

    # 4 cases where the target is
    angle=0
    # case 1
    if xEnd<xStart:
        angle = math.tan(GK / AK) * 180.0 / math.pi
        if yEnd<yStart:
            if move.getOrientation(clientID)<0:
                angle=90.0+angle+move.getOrientation(clientID)
            else:
                angle = 180.0-move.getOrientation(clientID)+90.0+angle
        if yEnd>yStart:
            print()
    # case 2:
    elif xEnd>xStart:
        angle = math.tan(GK / AK) * 180.0 / math.pi
        print()
    # case 3:
    else:
        #angle = math.tan(GK / AK) * 180.0 / math.pi
        print()


    return angle

def removeModel(clientID, name):
    res,toRemove=vrep.simxGetObjectHandle(clientID, name, vrep.simx_opmode_blocking)
    vrep.simxRemoveModel(clientID, toRemove, vrep.simx_opmode_oneshot)

if __name__ == "__main__": main()
