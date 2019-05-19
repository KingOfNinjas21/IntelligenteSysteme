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

goalName = "Goal"
LEFT_RAY_NINETY = 603
RIGHT_RAY_NINETY = 82
WIDTH_CUBE = 1
WIDTH_BOT = 50

def main():
    print ('Program started')
    emptybuf = bytearray()

    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 2000, 5)


    if clientID!=-1:
        print ('Connected to remote API server')

        # Start the simulation:
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait)

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



        # programmable space -------------------------------------------------------------------------------------------

        err, res, image = vrep.simxGetVisionSensorImage(clientID, youBotCam, 0, vrep.simx_opmode_buffer)
        image = colorDet.convertToCv2Format(image, res)
        imageC = image 
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #cv2.imshow("Current Image of youBot", imageC)
        #cv2.waitKey(0)
        #colorDet.test(imageC)

        found, prime_corners = cv2.findChessboardCorners(image, (3, 4))

        prime_corners = addOne(prime_corners)
        print(prime_corners)

        global_corners2 = [[-0.025, 0.125], [-0.025, 0.075], [-0.025, 0.025], [-0.075, 0.125],
                           [-0.075, 0.075], [-0.075, 0.025],
                           [-0.125, 0.125], [-0.125, 0.075], [-0.125, 0.025], [-0.175, 0.125],
                           [-0.175, 0.075], [-0.175, 0.025]]

        # print("Find homo: ",cv2.findHomography(global_corners2, prime_corners))

        # prime_corners = addOne(prime_corners)

        global_corners = [[-0.025, 0.125, 1.0], [-0.025, 0.075, 1.0], [-0.025, 0.025, 1.0], [-0.075, 0.125, 1.0],
                          [-0.075, 0.075, 1.0], [-0.075, 0.025, 1.0],
                          [-0.125, 0.125, 1.0], [-0.125, 0.075, 1.0], [-0.125, 0.025, 1.0], [-0.175, 0.125, 1.0],
                          [-0.175, 0.075, 1.0], [-0.175, 0.025, 1.0]]

        
        
        ego_corners  = []
        for gc in global_corners:
            newCorner = colorDet.globalToEgocentric(gc, clientID)
            ego_corners.append(newCorner)

        ego_corners = addOne(ego_corners)

        ego_corners = np.asarray(ego_corners)

        print(ego_corners)                
        

        A = getA(prime_corners, ego_corners)
        H = getH(A)

        point = np.dot(H, prime_corners[0])
        point = point / point[2]
        print(point)
        point = colorDet.egocentricToGlobal(point, clientID)
        print(point)

        blobs = colorDet.findAllBlobs(clientID, youBotCam, H)
        print(blobs)
        # end of programmable space --------------------------------------------------------------------------------------------



        # Stop simulation
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)

        # Close connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')


def addOne(matrix):
    new = []
    row = []
    for i in range(len(matrix)):
        row = np.append(matrix[i], 1)
        new.append(row)
    return np.asarray(new)


def getH(A):
    u, s, vh = np.linalg.svd(A, full_matrices=False)
    h = vh[8]
    H = np.reshape(h, (3, 3))
    return H


def getA(prime_corners, global_corners):
    A = []
    for i in range(len(prime_corners)):
        p1 = [-prime_corners[i][0], -prime_corners[i][1], -1, 0, 0, 0, prime_corners[i][0] * global_corners[i][0],
              prime_corners[i][1] * global_corners[i][0], global_corners[i][0]]
        p2 = [0, 0, 0, -prime_corners[i][0], -prime_corners[i][1], -1, global_corners[i][1] * prime_corners[i][0],
              global_corners[i][1] * prime_corners[i][1], global_corners[i][1]]

        A.append(np.asarray(p1))
        A.append(np.asarray(p2))

    return np.asarray(A)




#get the distance between two cubes - work in progress
def getDistanceBetweenCubes(posCube1, posCube2):
    xCube1 = posCube1[0]
    yCube1 = posCube1[1]

    xCube2 = posCube2[0]
    yCube2 = posCube2[1]

    xVec = abs(xCube1 - xCube2)
    yVec = abs(yCube1 - yCube2)

    distance = math.sqrt(xVec * xVec + yVec * yVec)

    return distance


def doTranslationOnVector(distX, distY, vector):
    newVec = vector
    newVec[0] += distX
    newVec[1] += distY

    return newVec


#method stub
def youBotFits(posCube1, posCube2):
    return

if __name__ == "__main__": main()
