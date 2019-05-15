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
        x, y = egocentricToGlobal([1.0, 1.0], clientID)
        print(globalToEgocentric(x, y, clientID))

        '''
        err, res, image = vrep.simxGetVisionSensorImage(clientID, youBotCam, 0, vrep.simx_opmode_buffer)
        image = colorDet.convertToCv2Format(image, res)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        found, prime_corners = cv2.findChessboardCorners(image,(3,4))

        prime_corners_one = addOne(prime_corners)

        global_corners = [[-0.025, 0.125, 1], [-0.025, 0.075, 1], [-0.025, 0.025, 1], [-0.075, 0.125, 1], [-0.075, 0.075, 1], [-0.075, 0.025, 1],
                          [-0.125, 0.125, 1], [-0.125, 0.075, 1], [-0.125, 0.025, 1], [-0.175, 0.125, 1], [-0.175, 0.075, 1], [-0.175, 0.025, 1]]

        #cv2.imshow("Penis", image)
        #cv2.waitKey(0)

        cvHomo, mask = cv2.findHomography(prime_corners_one, np.array(global_corners))
        
        
        # print and calc some cv2.findHomography matrizes
        print(cvHomo)
        
        print("\n\n")

        point = np.dot(cvHomo, prime_corners_one[0])
        
        print(point / point[2])
        
        print("\n\n")
        
        print(prime_corners_one)
        
        print("\n\n")



        # calc our own homogene matirix
        A = calcHomgenMatrix(prime_corners_one, global_corners)
        cA = A
        print("Neue A matrix", A)
        A = np.delete(A, 8, axis=1)
        A = np.linalg.pinv(A)

        d = np.reshape(prime_corners, (24, 1))
        print("d: ", d)

        h = np.dot(A, d)
        h = np.append(h, 1)

        h = np.reshape(h, (3, 3))
        h = h/h[2, 2]
        print("h matrix: ", h)
        # calc via pinvc

        

        u, s, vh = np.linalg.svd(cA)

        vh = np.transpose(vh)

        vh = vh[:, 8]
        vh = np.reshape(vh, (3, 3))
        vh = vh / vh[2,2]
        '''

        
        '''
        ourPoint = np.dot(vh, prime_corners[0])
        
        print(ourPoint / ourPoint[2])
        
        print("\n\n")
        print(vh)
        '''
        
        #print(np.dot(prime_corners, vh))
        
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


def calcHomgenMatrix(prime_corners, global_corners):
    A = np.empty((1, 9))
    for i in range(len(prime_corners)):
        '''dot11 = np.dot(np.transpose(global_corners[i]), -prime_corners[i][2])
        dot12 = np.dot(np.transpose(global_corners[i]), prime_corners[i][1])

        a1 = np.concatenate((np.zeros(3), dot11, dot12[:-1]))


        dot21 = np.dot(np.transpose(global_corners[i]), prime_corners[i][2])
        dot22 = np.dot(np.transpose(global_corners[i]), -prime_corners[i][0])

        a2 = np.concatenate((dot21, np.zeros(3), dot22[:-1]))'''

        '''
        dot31 = np.dot(np.transpose(global_corners[i]), -prime_corners[i][1])
        dot32 = np.dot(np.transpose(global_corners[i]), prime_corners[i][0])

        a3 = np.concatenate((dot31, dot32, np.zeros(3)))
        '''

        #A = np.append(A, [a1], axis=0)
        #A = np.append(A, [a2], axis=0)
        #A = np.append(A, [a3], axis=0)
        row1 = [0.0, 0.0, 0.0, global_corners[i][0], global_corners[i][1], 1.0, -global_corners[i][0]*prime_corners[i][1], -prime_corners[i][1]*global_corners[i][1], global_corners[i][1]]
        row2 = [global_corners[i][0], global_corners[i][1], 1.0, 0.0, 0.0, 0.0, -global_corners[i][0] * prime_corners[i][0], -global_corners[i][1] * prime_corners[i][0], global_corners[i][0]]

        A = np.append(A, [row2], axis=0)
        A = np.append(A, [row1], axis=0)
    #np.delete(A, 1, axis=0)

    return A[1:]

def egocentricToGlobal(ego, clientID):
    x = ego[0]
    y = ego[1]
    pos, orient = move.getPos(clientID)
    alpha = move.getOrientation(clientID)/math.pi*180.0
    move.printPos(clientID)
    rotationMatrix = [[math.cos(-alpha), -math.sin(-alpha)],
                      [math.sin(-alpha), math.cos(-alpha)]]
    newVec = np.dot(np.array(rotationMatrix), np.array([x, y]))

    x = newVec[0]
    y = newVec[1]

    xBot = pos[0]
    yBot = pos[1]



    return x+xBot, y+yBot

def globalToEgocentric(globalX, globalY, clientID):
    pos, orient = move.getPos(clientID)
    egoVec = [globalX-pos[0], globalY-pos[1]]

    alpha = move.getOrientation(clientID)/math.pi*180.0

    rotationMatrix = [[math.cos(alpha), -math.sin(alpha)],
                      [math.sin(alpha), math.cos(alpha)]]
    newVec = np.dot(np.array(rotationMatrix), np.array(egoVec))

    return newVec[0], newVec[1]
if __name__ == "__main__": main()
