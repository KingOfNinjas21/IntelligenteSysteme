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


        # initialize sensor and get sensor handles:
        rangeSen.initializeSensor(clientID)
        hokuyo = rangeSen.getSensorHandles(clientID)
        res, rayHit = move.headTowardsModel(clientID, "Goal", hokuyo)
        move.wallOrient(clientID,hokuyo, rayHit)
        res, aux, auxD = vrep.simxReadVisionSensor(clientID, hokuyo[0], vrep.simx_opmode_streaming)
        res, aux, auxD = vrep.simxReadVisionSensor(clientID, hokuyo[1], vrep.simx_opmode_streaming)        


        # Stop simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')


def followBoundary(clientID):
    # initialize sensor
    range.initializeSensor(clientID)
    # Get sensor handle:
    hokuyo = rangeSen.getSensorHandles(clientID)
    
    #eighter go left or right
	
	#while (not detectClearPath(clientID)):
    #    rangeData = range.getSensorData(clientID, hokuyo[1])
    # calculate diraction to move
	# Two closest sensor values to the robot estimate the closest wall
		
	# follow boundary
	
def detectClearPath(clientID):
    # todo: realize clear path
	
	return False


def removeModel(clientID, name):
    res,toRemove=vrep.simxGetObjectHandle(clientID, name, vrep.simx_opmode_blocking)
    vrep.simxRemoveModel(clientID, toRemove, vrep.simx_opmode_oneshot)

if __name__ == "__main__": main()
