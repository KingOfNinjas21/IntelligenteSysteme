# Introduction to Intelligent and Autonomus Systems, UIBK, 2017
#contact:senka.krivic@uibk.ac.at
# 

import vrep
import numpy as np
import time
import math
import movementFunctions as move
import rangeSensorFunctions as rangeSen

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


        # initialize sensor
        rangeSen.initializeSensor(clientID)

        # Get sensor handle:
        hokuyo = rangeSen.getSensorHandles(clientID)
        #move.forward(0.1, clientID)
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

        #move.forwardUntilObstacle(3, clientID, hokuyo[0])
        #move.rotate(90,clientID, True)
        #headTowardsModel(clientID, "conferenceChair", hokuyo[0])
        res, aux, auxD = vrep.simxReadVisionSensor(clientID, hokuyo[0], vrep.simx_opmode_streaming)
        res, aux, auxD = vrep.simxReadVisionSensor(clientID, hokuyo[1], vrep.simx_opmode_streaming)        
        #move.forward(0.2, clientID)
        #result = rangeSen.getSensorData(clientID, hokuyo)
        #print(result)

        #for i in range(len(result)):
        #    print(result[i])
        #for i in range(len(result)):
        #     print(result[i][0])
        #print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")

        #for i in range(len(result)):
        #    print(result[i][1])
        #print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")

        #for i in range(len(result)):
        #    print(result[i][2])
        #print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")

        #for i in range(len(result)):
        #    print(result[i][3])
        #print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")


        # Stop simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)
        headTowardsModel(clientID, "Goal", hokuyo[0])
        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')


def followBoundary(clientID):
	# initialize sensor
    rangeSen.initializeSensor(clientID)
    # Get sensor handle:
    hokuyo = rangeSen.getSensorHandles(clientID)
    
    #eighter go left or right
	
    while (not detectClearPath(clientID)):
        rangeData = rangeSen.getSensorData(clientID, hokuyo[1])
        # calculate diraction to move
        # Two closest sensor values to the robot estimate the closest wall

        # follow boundary
	
def detectClearPath(clientID):
	# todo: realize clear path
	
	return null


def headTowardsModel(clientID, modelName, rangeSensorHandle):
    res, objHandle = vrep.simxGetObjectHandle(clientID, modelName, vrep.simx_opmode_oneshot_wait)

    targetPosition = vrep.simxGetObjectPosition(clientID, objHandle, -1, vrep.simx_opmode_oneshot_wait)
    xTarget = targetPosition[1][0]
    yTarget = targetPosition[1][1]
    print ("{}: x= {}, y= {}" .format(modelName,xTarget,yTarget))
    pos, ori = move.getPos(clientID)

    targetOrientation = calcTargetOrient(clientID, pos[0], pos[1], xTarget, yTarget)
    print("Orientation of target: ", targetOrientation)

    move.rotateUntilOrientation(clientID, targetOrientation, True)
    print("moved")
    '''
    
    print("Angle: ", angle)
    if angle>0:
        move.rotate(angle, clientID, True)
    else:
        move.rotate(angle, clientID, False)
    
    '''
    print(pos[0], " ", pos[1])
    dist = calcDistanceToTarget(pos[0], pos[1], xTarget, yTarget)
    move.forwardUntilObstacle(dist, clientID, rangeSensorHandle)

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

def calcTargetOrient(clientID, xStart, yStart, xEnd, yEnd):
    GK = float(yEnd-yStart)
    AK = float(xEnd-xStart)

    # 4 cases where the target is
    angle=0
    # case 1
    if xEnd<xStart:
        angle = math.atan2(GK, AK) * 180.0 / math.pi
        if yEnd<yStart:
            targetOrient = 90.0 + angle
        if yEnd>yStart:
            targetOrient = 90.0 - angle
    elif xEnd>xStart:
        angle = math.tan(GK / AK) * 180.0 / math.pi
        if yEnd < yStart:
            targetOrient = - 90.0 - angle
        if yEnd > yStart:
            targetOrient = - 90.0 + angle
    # case 3:
    else:
        #angle = math.tan(GK / AK) * 180.0 / math.pi
        print()


    return angle


def removeModel(clientID, name):
    res,toRemove=vrep.simxGetObjectHandle(clientID, name, vrep.simx_opmode_blocking)
    vrep.simxRemoveModel(clientID, toRemove, vrep.simx_opmode_oneshot)

if __name__ == "__main__": main()
