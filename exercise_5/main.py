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
import aStar

def main():
    print ('Program started')
    emptybuf = bytearray()

    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 2000, 5)


    if clientID!=-1:
        print ('Connected to remote API server')

        # Start the simulation:
        vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)

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

        print("Begin calculation of H-matrix, please wait ...")
        err, res, image = vrep.simxGetVisionSensorImage(clientID, youBotCam, 0, vrep.simx_opmode_buffer)
        image = colorDet.convertToCv2Format(image, res)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        found, prime_corners = cv2.findChessboardCorners(image, (3, 4))

        prime_corners = addOne(prime_corners)

        # gCX are the world coordinates for the points on the chessboards which are later used to construct the h-matrix
        gCX = [[0.075, 0.55, 1.0], [0.075, 0.5, 1.0], [0.075, 0.45, 1.0], [0.025, 0.55, 1.0], [0.025, 0.5, 1.0], [0.025, 0.45, 1.0], [-0.025, 0.55, 1.0], [-0.025, 0.5, 1.0], [-0.025, 0.45, 1.0], [-0.075, 0.55, 1.0], [-0.075, 0.5, 1.0], [-0.075, 0.45, 1.0]]

        # convert all global corners of the chessboard in egocentric world space
        ego_corners  = []
        for gc in gCX:
            newCorner = colorDet.globalToEgocentric(gc, clientID)
            ego_corners.append(newCorner)

        # add a 1 in every row (globalToEgocentric only returns x,y coordinates
        ego_corners = addOne(ego_corners)

        # convert ego_corners in numpy array
        ego_corners = np.asarray(ego_corners)

        # calculate H-matrix
        A = getA(prime_corners, ego_corners)
        H = getH(A)
        print("H-matrix calculated")

        blobs = colorDet.findAllBlobs(clientID, youBotCam, H)
        obstacleList = []
        for b in blobs:
            obstacleList.append(b[0])

        print("Found blobs:")

        # sort blob list and print it
        blobs = sortBlobsByRad(blobs)
        for blob in blobs:
            print("Coordinate: ", blob[0], " Upper and lower Bound of the color: ", blob[1])

        # get position of youBot and goal
        roboPos, ori = move.getPos(clientID)
        res, objHandle = vrep.simxGetObjectHandle(clientID, "goal", vrep.simx_opmode_oneshot_wait)
        targetPosition = vrep.simxGetObjectPosition(clientID, objHandle, -1, vrep.simx_opmode_oneshot_wait)
        targetPosition = targetPosition[1][:2]

        # drive to the goal with obstacles ahead
        driveThroughPath(obstacleList, roboPos[:2], targetPosition, clientID)

        # end of programmable space --------------------------------------------------------------------------------------------

        # Stop simulation
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)

        # Close connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')

# adds a one at the end of every row
def addOne(matrix):
    new = []
    for i in range(len(matrix)):
        row = np.append(matrix[i], 1)
        new.append(row)
    return np.asarray(new)

# retrieve H-matrix via the A-matrix
def getH(A):
    # the matrix we want is the last row of vh which needs then to be reshaped
    u, s, vh = np.linalg.svd(A, full_matrices=False)
    h = vh[8]
    H = np.reshape(h, (3, 3))
    return H

# retrieve A-matrix via prime corners and global corners
# prime_corners = points in screen (picture) space
# global_corners = points in world space
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

# calculates a path from the youBotPos to the goalPos and drives to it
def driveThroughPath(obstacleCoordinates, youBotPos, goalPos, clientID):
    # retrieve an AStar_Solver object
    a = aStar.AStar_Solver(youBotPos, goalPos, obstacleCoordinates)
    #obstacles = [[0.5, 0.75], [1.0, 1.0], [1.5, 1], [1.5, 0.5], [1.75, 0.25], [2.0, 0], [2.0, -0.5], [3.0, 0.5]]
    print("Starting AStar algorithm...")
    #a = aStar.AStar_Solver([0.0, 0.0], [2.5, 0.5], obstacleCoordinates)
    a.Solve()
    print("Found path: ")
    for p in a.path:
        print(p)

    print("\n")

    # go through path list and move from point to point
    for p in range(1, len(a.path)):
        print("next target: ", a.path[p])
        move.moveToCoordinate(a.path[p][0], a.path[p][1], clientID)

# sort the blobs from right to left (selectionSort)
def sortBlobsByRad(blobs):
    blobsCopy = blobs[:]

    blobsSorted = [([],([],[]))]

    while len(blobsCopy) > 0:
        min = blobsCopy[0]
        for blob in blobsCopy:
            if getBlobXAngle(blob) < getBlobXAngle(min):
                min = blob

        blobsSorted.append(min)
        blobsCopy.remove(min)

    return blobsSorted[1:]


def getBlobXAngle(blob):
    return math.atan(blob[0][1] / blob[0][0])


if __name__ == "__main__": main()
