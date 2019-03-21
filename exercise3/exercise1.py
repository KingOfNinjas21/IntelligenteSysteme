# Introduction to Intelligent and Autonomus Systems, UIBK, 2017
#contact:senka.krivic@uibk.ac.at
# 

import vrep
import numpy as np
import time
import math
import movementFunctions as move


def main():
    print ('Program started')
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,True,True, 2000,5)
    
    removeModel(clientID, "youBot2")    

    if clientID!=-1:
        print ('Connected to remote API server')
    
        emptyBuff = bytearray()
    
        # Start the simulation:
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait)
    
        # initiaize robot
        # Retrieve wheel joint handles:
        wheelJoints=np.empty(4, dtype=np.int); wheelJoints.fill(-1) #front left, rear left, rear right, front right
        res,wheelJoints[0]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fl',vrep.simx_opmode_oneshot_wait)
        res,wheelJoints[1]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rl',vrep.simx_opmode_oneshot_wait)
        res,wheelJoints[2]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fr',vrep.simx_opmode_oneshot_wait)
        res,wheelJoints[3]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rr',vrep.simx_opmode_oneshot_wait)

        move.printPos(clientID)

        # start Hokuyo sensor
        res = vrep.simxSetIntegerSignal(clientID, 'handle_xy_sensor', 2, vrep.simx_opmode_oneshot);

        # display range sensor beam
        vrep.simxSetIntegerSignal(clientID, 'displaylasers', 1, vrep.simx_opmode_oneshot);

        #Get sensor handle:
        res, hokuyo1 = vrep.simxGetObjectHandle(clientID, 'fastHokuyo_sensor1', vrep.simx_opmode_oneshot_wait)
        res, hokuyo2 = vrep.simxGetObjectHandle(clientID, 'fastHokuyo_sensor2', vrep.simx_opmode_oneshot_wait)

        # retrieve sensor data, the streaming opmode is at least one time necessary, afterwards the buffer mode is enough to get data from range senor
        res, aux, auxD = vrep.simxReadVisionSensor(clientID, hokuyo1, vrep.simx_opmode_streaming)
        print "Streaming mode: ", auxD
        res, aux, auxD = vrep.simxReadVisionSensor(clientID, hokuyo1, vrep.simx_opmode_buffer)
        print "Buffer mode: ", auxD

        #res, aux, auxD = vrep.simxReadVisionSensor(clientID, hokuyo1, vrep.simx_opmode_buffer)
        # print "Buffer mode: ", auxD[1]
        #positions = transformInMatrix(auxD)
        #print positions

        move.rotateDegrees(90.0, clientID, 0.25)
        move.forward(3.0, 0.5, clientID)
        move.forward(-3.0, 0.5, clientID)

        #rotateDegrees(180, clientID, 0.25)

        # Stop simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')


def transformInMatrix(auxD):
    length = int(auxD[1][1])
    #print length
    width =  int(auxD[1][0])
    #print width
    result = [[0,0,0,0]]*(length*width)
    #print result
    k=2

    #every entry in result represents x,y,z,distance of every point detected of the range sensor

    for i in range(length*width) :
        for j in range(4) :
            result[i][j] = auxD[1][k]
            k+=1
    return result

def removeModel(clientID, name):
    res,toRemove=vrep.simxGetObjectHandle(clientID, name, vrep.simx_opmode_blocking)
    vrep.simxRemoveModel(clientID, toRemove, vrep.simx_opmode_oneshot)

if __name__ == "__main__": main()
