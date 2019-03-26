# start Hokuyo sensor
import vrep
import numpy as np
import time
import math
import movementFunctions as move
import pdb

def initializeSensor(clientID):
    # start Hokuyo sensor
    res = vrep.simxSetIntegerSignal(clientID, 'handle_xy_sensor', 2, vrep.simx_opmode_oneshot);

    # display range sensor beam
    vrep.simxSetIntegerSignal(clientID, 'displaylasers', 1, vrep.simx_opmode_oneshot);

    # Get sensor handle:
    hokuyo = getSensorHandles(clientID)

    # initialize sensor to retrieve data
    # to retrieve sensor data, the streaming opmode streaming is at least one time necessary
    res, aux, auxD = vrep.simxReadVisionSensor(clientID, hokuyo[0], vrep.simx_opmode_streaming)
    res, aux, auxD = vrep.simxReadVisionSensor(clientID, hokuyo[1], vrep.simx_opmode_streaming)

def getSensorHandles(clientID):
    # Get sensor handle:
    res, hokuyo1 = vrep.simxGetObjectHandle(clientID, 'fastHokuyo_sensor1', vrep.simx_opmode_oneshot_wait)
    res, hokuyo2 = vrep.simxGetObjectHandle(clientID, 'fastHokuyo_sensor2', vrep.simx_opmode_oneshot_wait)

    return hokuyo1, hokuyo2

# only use this function if sensor data was initialized via initializeSensorData
def getSensorData(clientID, sensorHandles):
    res, aux, auxD = vrep.simxReadVisionSensor(clientID, sensorHandles[0], vrep.simx_opmode_buffer)
    result1 = transformInMatrix(auxD)
    result1 = convertTransformedDataSensor1(clientID, sensorHandles[0], result1)

    res, aux, auxD = vrep.simxReadVisionSensor(clientID, sensorHandles[1], vrep.simx_opmode_buffer)
    result2 = transformInMatrix(auxD)
    result2 = convertTransformedDataSensor1(clientID, sensorHandles[1], result2)

    return result1+result2

def transformInMatrix(auxD):
    if not auxD :
        return []
    length = int(auxD[1][1])

    width =  int(auxD[1][0])

    result = np.zeros((length* width, 4))

    k=2

    #pdb.set_trace()

    #every entry in result represents x,y,z,distance of every point detected of the range sensor
    for i in range(length*width) :
        for j in range(0,4) :
            result[i][j] = auxD[1][k]
            k+=1

    return result

def convertTransformedDataSensor1(clientID, sensorHandle, dataMatrix):
    base_orient = vrep.simxGetObjectOrientation(clientID, sensorHandle, -1, vrep.simx_opmode_oneshot_wait)
    sensorAngle = base_orient[1][2]

    Rx = [[1, 0, 0],
          [0, math.cos(sensorAngle), -math.sin(sensorAngle)],
          [0, math.sin(sensorAngle), math.cos(sensorAngle)]]

    Ry = [[math.cos(sensorAngle), 0, math.sin(sensorAngle)],
          [0, 1, 0],
          [-math.sin(sensorAngle), 0, math.cos(sensorAngle)]]

    Rz = [[math.cos(sensorAngle), -math.sin(sensorAngle), 0],
          [math.sin(sensorAngle), math.cos(sensorAngle), 0],
          [0, 0, 1]]
    resultTemp = multiplyMatrices(Rx, Ry)
    resultTemp = multiplyMatrices(resultTemp, Rz)

    result = [[0]*4 for i in range(len(dataMatrix))]
    for i in range(len(dataMatrix)):
        result[i] = multiplyMatrixVector(resultTemp, dataMatrix[i])
    return result

def multiplyMatrixVector(matrix, vector):
    result = [0]*len(vector)
    result[0] = matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2]
    result[1] = matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2]
    result[2] = matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2]
    result[3] = vector[3]
    return result

def multiplyMatrices(m1, m2):
    #result = [[0]*len(m1)]*len(m2[0])
    result = np.zeros((len(m1), len(m2)))
    for i in range(len(m1)):
        # iterate through columns of Y
        for j in range(len(m2[0])):
            # iterate through rows of Y
            for k in range(len(m2)):
                result[i][j] += m1[i][k] * m2[k][j]
    return result