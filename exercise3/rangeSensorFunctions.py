# start Hokuyo sensor
import vrep
import numpy as np
import time
import math

def initializeSensor(clientID):
    # start Hokuyo sensor
    startHokuyoSensor(clientID)

    # display range sensor beam
    displaySensorBeam(clientID)

    # Get sensor handle:
    hokuyo = getSensorHandles(clientID)

    # initialize sensor to retrieve data
    initializeSensorData(clientID, hokuyo[1])


def startHokuyoSensor(clientID):
    res = vrep.simxSetIntegerSignal(clientID, 'handle_xy_sensor', 2, vrep.simx_opmode_oneshot);

def displaySensorBeam(clientID):
    vrep.simxSetIntegerSignal(clientID, 'displaylasers', 1, vrep.simx_opmode_oneshot);

def getSensorHandles(clientID):
    # Get sensor handle:
    res, hokuyo1 = vrep.simxGetObjectHandle(clientID, 'fastHokuyo_sensor1', vrep.simx_opmode_oneshot_wait)
    res, hokuyo2 = vrep.simxGetObjectHandle(clientID, 'fastHokuyo_sensor2', vrep.simx_opmode_oneshot_wait)

    return hokuyo1, hokuyo2


# to retrieve sensor data, the streaming opmode streaming is at least one time necessary
def initializeSensorData(clientID, sensorHandle):
    res, aux, auxD = vrep.simxReadVisionSensor(clientID, sensorHandle, vrep.simx_opmode_streaming)

# only use this function if sensor data was initialized via initializeSensorData
def getSensorData(clientID, sensorHandle):
    res, aux, auxD = vrep.simxReadVisionSensor(clientID, sensorHandle, vrep.simx_opmode_buffer)
    result = transformInMatrix(auxD)
    return result

def transformInMatrix(auxD):
    if not auxD :
        return []
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
