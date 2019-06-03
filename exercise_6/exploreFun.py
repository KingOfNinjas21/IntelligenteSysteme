from queue import Queue
import bugFunctions as bug
import movementFunctions as move
import vrep
import time
import rangeSensorFunctions as rangeSensor
import math
import colorDetection as colorDet
import constants as c
import numpy as np
import cv2

def initPath():
    queue = Queue()
    explorePath = [(3.0, 1.0), (6.5, 3.0), (4.4, 5.9), (-4.0, 6.6), (-4.4, -1.0), (-3.6, -4.0), (-5.0, -4.6), (+1.5, -5.0), (3.5, -5.0), (3.5, -2.0), (-0.3, -2.0)]
    for i in range(len(explorePath)):
        print ("added to queue: ", explorePath[i])
        queue.put(explorePath[i])
    return queue

def init_state(youBotCam, clientID):
    path = initPath()

    # init H-Matrix
    return 2, path, colorDet.get_H_matrix(c.gCX, youBotCam, clientID)



'''
Needed adjusted movement functions 
'''

# adjusted of distbug algorithm
def distB(clientID, sensorHandles, path):
    if not path.empty():
        targetPos = path.get()
        nextState = -1
        isNotGoal = True
        while (isNotGoal):
            isNotGoal, hitRay = headTowardsModel(clientID, targetPos, sensorHandles)

            # if headTowardsModel returned False it means it successfully got to the goal point
            if (not isNotGoal):
                print("REACHED GOAL")
                nextState = 2  # 2 = find all blobs
                return nextState

            # if there is no chair and headTowardsModel returned True -> follow the boundary
            else:
                # orient to wall to have a better start when following a wall
                isRight = bug.wallOrient(clientID, sensorHandles, hitRay, False)
                bug.followBoundary(clientID, sensorHandles, isRight)

            print("Bot is in goal: {}".format(not isNotGoal))
    else:
        nextState = 0 # 0 = finished with programm
    return nextState


# Get to the targetPos [x,y]
# returns false if the bot successfully reached the target and true if the bot encountered an obstacle
def headTowardsModel(clientID, targetPos, rangeSensorHandles):
    print("HeadTowardsModel: start")
    print("Next goal: x={}, y={}".format(targetPos[0], targetPos[1]))
    # get the needed position information's of youBot and modelName (target)
    xTarget = targetPos[0]
    yTarget = targetPos[1]
    pos, ori = move.getPos(clientID)

    # calculate the target orientation based on the current youBot position
    targetOrientation = bug.calcTargetOrient(clientID, pos[0], pos[1], xTarget, yTarget)

    # rotate towards modelName until target orientation is reached
    rotateUntilOrientation(clientID, targetOrientation)

    #dirve for dist meters or stop when encountering an obstacle
    case, hit = forwardUntilObstacleAnywhere(targetPos, clientID, rangeSensorHandles)

    return case, hit

# drives forward to the target position long as there is no obstacle around the bot within the distance of 0.3
# around the bot means ca +90 and -90 degrees(from the front)
# returns true if the bot encounterd an obstacle, and false if the bot reached the target
def forwardUntilObstacleAnywhere(targetPos, clientID, rangeSensorHandles):
    # set velocety to 0
    move.setWheelVelocity(clientID, 0)
    # start moving
    move.startMoving(clientID)
    # continuously check traveled distance
    distance = 0.0
    dt = 0.0
    maxFalseAngle = 5
    x, y = move.getPos(clientID)[0][:-1]
    xCurrent, yCurrent = move.getPos(clientID)[0][:-1]
    stop = False
    hit = 0
    while not isSamePoint(targetPos, [xCurrent, yCurrent]) and stop != True:

        # get range sensor data as list of x,y,z,distance
        rangeData = rangeSensor.getSensorData(clientID, rangeSensorHandles)
        # range sensor angle is estimated for about 250 degrees
        for i in range(95, 588): #95 and 588 are estimated values for data from range sensor which are between -90 and 90 degrees (0 is front)
            if rangeData[i][3]<=0.3:
                stop=True
                hit = i
                break
        time.sleep(1.0e-06)         # problems with very small time slices -> little delay (if you have a bad angle calculation on your pc try to change this value)
        end = time.time()

        xCurrent, yCurrent = move.getPos(clientID)[0][:-1]
        targetOrientation = move.calcTargetOrient(xCurrent, yCurrent, targetPos[0], targetPos[1])
        if abs(move.getOrientation(clientID)-targetOrientation)>maxFalseAngle:
            rotateUntilOrientation(clientID, targetOrientation)
            move.startMoving(clientID)
    # stop moving
    move.setWheelVelocity(clientID, 0)

    return(stop, hit)


# This function compares two points with some latitude
def isSamePoint(pointA, pointB):
    latituteRadius = 0.2

    if move.getDistanceBetweenPoints(pointA, pointB) <= latituteRadius:
        return True

    return False

# rotates the youBot until he reached a certain orientation value
# copied from movementFunctions but changed only the rotation velocity
def rotateUntilOrientation(clientID, targetOrient):
    move.setWheelVelocity(clientID, 0)
    # get needed youBot information's
    rotationVel = 0.8
    wheelJoints = move.getWheelJoints(clientID)
    currentOrient = move.getOrientation(clientID)
    safty = False
    #print("Orientations for rotation: Current Orientation = {} , Target Orientation = {}" .format(currentOrient, targetOrient))

    # The following are all possibilities that can happen when rotating until a certain orientation

    if(currentOrient > 0 and targetOrient < 0 and abs(targetOrient - currentOrient)> 180):
        rotationVel *= -1
        move.startRotating(clientID, rotationVel)
        if(targetOrient + 1) > 180:
            safty = True
        while not (targetOrient + 5.0 > currentOrient and currentOrient > targetOrient -5.0): #fails at 180/-180
            time.sleep(0.05)
            currentOrient = move.getOrientation(clientID)
            if(currentOrient < -175 and safty == True):
                break

    elif(currentOrient > 0 and targetOrient < currentOrient):
        rotationVel *= 1
        move.startRotating(clientID, rotationVel)
        if(targetOrient - 1) < -180:
            safty = True
        while targetOrient < currentOrient: #fails at 0
            time.sleep(0.05)
            currentOrient = move.getOrientation(clientID)
            if(currentOrient > 175) and safty == True:
                break

    elif(currentOrient > 0 and targetOrient > currentOrient):
        rotationVel *= -1
        move.startRotating(clientID, rotationVel)
        if(targetOrient + 1) > 180:
            safty = True
        while targetOrient > currentOrient: #fails at 180/-180
            time.sleep(0.05)
            currentOrient = move.getOrientation(clientID)
            if(currentOrient < -175 and safty == True):
                break


    elif(currentOrient < 0 and targetOrient > 0 and abs(targetOrient-currentOrient)>180):
        rotationVel *= 1
        move.startRotating(clientID, rotationVel)
        if(targetOrient - 1) < -180:
            safty = True
        while not (targetOrient + 5.0 > currentOrient and targetOrient - 5.0 < currentOrient): #fails at 0
            time.sleep(0.05)
            currentOrient = move.getOrientation(clientID)
            if(currentOrient > 175) and safty == True:
                break
    elif(currentOrient < 0 and targetOrient < currentOrient):
        rotationVel *= 1
        move.startRotating(clientID, rotationVel)
        if(targetOrient - 1) < -180:
            safty = True
        while targetOrient < currentOrient: #fails at 0
            time.sleep(0.05)
            currentOrient = move.getOrientation(clientID)
            if(currentOrient > 175) and safty == True:
                break

    else:
        rotationVel *= -1
        move.startRotating(clientID, rotationVel)
        if(targetOrient + 1) > 180:
            safty = True
        while targetOrient > currentOrient: #fails at 180/-180
            time.sleep(0.05)
            currentOrient = move.getOrientation(clientID)
            if(currentOrient < -175 and safty == True):
                break



    # stop moving
    move.setWheelVelocity(clientID, 0)

    return

# moves youBot to the nex blob in the blob list
def getToNextBlob(clientID, blobsList, visitedBlobsList):
    print("Start getting to the next blob")
    nextBlob = blobsList[0]
    visitedBlobsList.append(nextBlob)

    # get robot pos
    roboPos = move.getPos(clientID)

    # move to the position of next blob
    xA = nextBlob[0] - roboPos[0]
    yA = nextBlob[1] - roboPos[1]
    lenA = math.sqrt(xA * xA + yA * yA)

    shortenXA = xA * (1 - (c.maxDistToBlock / lenA))
    shortenYA = yA * (1 - (c.maxDistToBlock / lenA))

    # move forward until block dist - constants.maxDistToBlock
    move.moveToCoordinate(shortenXA, shortenYA, clientID)

    print("End getting to the next blob")
    return 4, blobsList, visitedBlobsList   # grab blob

# moves youBot back to the posBeforeMoveToBlob point
def moveBack(clientID, posBeforeMoveToBlob):
    print("Start moving back to " + posBeforeMoveToBlob)
    # TODO: move back
    #  maybe use the following for moving back: ex.headTowardsModel(clientID, posBeforeMoveToBlob, hokuyo)

    state = 3 # 3 = move to next blob
    print("End moving back")
    return state

def pdControl(clientID, youBotCam, goal):
    print("begin PD control")
    nextState = 4
    wheelJoints = move.getWheelJoints(clientID)
    err, res, image = vrep.simxGetVisionSensorImage(clientID, youBotCam, 0, vrep.simx_opmode_buffer)
    cv2Image = colorDet.convertToCv2Format(image, res)

    #cv2.imshow("Current Image of youBot", cv2Image)
    #cv2.waitKey(0)

    cor = getRedBlobPicture(cv2Image)
    preCor = cor
    rotVel = 0.5
    dt = 0.0
    forwBackVel = 0.5
    leftRightVel = 0.5
    while not velOk(forwBackVel, leftRightVel):
        # retrieve new coordinate of red blob
        err, res, image = vrep.simxGetVisionSensorImage(clientID, youBotCam, 0, vrep.simx_opmode_buffer)
        cv2Image = colorDet.convertToCv2Format(image, res)

        cor = getRedBlobPicture(cv2Image)

        # calculate new velocities
        forwBackVel = 0.1 * (cor[0][0] - goal[0]) - 0.002 * (cor[0][0] - preCor[0][0])
        leftRightVel = 0.08 * (cor[0][1] - goal[1]) - 0.001 * (cor[0][1] - preCor[0][1])
        print("Forward: ", forwBackVel)
        print("Left: ", leftRightVel)
        preCor = cor

        #vf, vr, w0 = move.formatVel(forwBackVel, leftRightVel, rotVel)

        # update velocities


        move.setWheelVelocity(clientID, 0.0)
        #time.sleep(1)
        vrep.simxPauseCommunication(clientID, True)
        for i in range(0, 4):
            vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], move.wheelVel(forwBackVel/10.0, leftRightVel/10.0, 0.0)[i],
                                            vrep.simx_opmode_oneshot)
        vrep.simxPauseCommunication(clientID, False)
    move.setWheelVelocity(clientID, 0.0)
    cv2.imshow("Current Image of youBot", cv2Image)
    cv2.waitKey(0)
    print("velocity is nearly zero")
    print("begin PD control")
    return nextState

def velOk(forwBackVel, leftRightVel):
    tolerance = 0.1
    forwBackOk = forwBackVel < tolerance and forwBackVel > -tolerance
    leftRightOk = leftRightVel < tolerance and leftRightVel > -tolerance
    return forwBackOk and leftRightOk
'''
Color Detection Functions
'''

# finds all new blobs, that are not already visited
def findBlobs(clientID, youBotCam, H, currentBlobsList, visitedBlobsList):
    blobs = findAllBlobs(clientID, youBotCam, H)
    #visitedBlobsList, visitedBlobsQueue = getListFromQueue(visitedBlobsQueue)
    obstacleList = []

    print("Start checking which blobs are new: ")
    for b in blobs:
        isNew = True

        for visited in visitedBlobsList:
            if isSameBlob(b[0], visited):
                isNew = False
        if isNew:
            print("Found new blob: {}".format(b[0]))
            obstacleList.append(b[0])
    print("End checking for new blobs")

    for blob in obstacleList:
        # TODO maybe check here if the blob is too far away -> if so don't add it to the list
        #  (as the bot moves on, he will later be closer to the blob)
        #  one reason todo so is that if the blob is too far away, the blobs position is highly incorrect
        currentBlobsList.append(blob)

    nextState = 3  # 3 = move to next blob
    if len(currentBlobsList)==0:
        nextState = 6 # 6 = move to next goal

    return nextState, currentBlobsList

# finds all blobs 360 degrees around the youBot
# returns a list with all found blobs
def findAllBlobs(clientId, youBotCam, homoMatrix):
    print("Start find all blobs around youBot")
    currentDegree = 0
    degreePerPic = 60.0
    amountPics = math.ceil(360.0/degreePerPic)
    blobList = []
    # we will rotate for 360 degree to spot all blobs near the bot
    i = 0

    while (i < amountPics):
        i += 1

        err, res, image = vrep.simxGetVisionSensorImage(clientId, youBotCam, 0, vrep.simx_opmode_buffer)

        if err == vrep.simx_return_ok:
            # do some image stuff ----------------------------------------------------------------------------------

            cv2Image = colorDet.convertToCv2Format(image, res)

            tempBlobs = colorDet.getBlobsGlobal(cv2Image, homoMatrix, clientId)

            count = 0
            for tb in tempBlobs:
                count = 0
                for b in blobList:
                    if isSameBlob(tb[0], b[0]) and tb[1] == b[1]:
                        count = count + 1

                if count == 0:
                    print("Added", tb, " to blobList")
                    blobList.append(tb)

        move.rotate(degreePerPic, clientId, True)
    print("End find all blobs")
    return blobList

# detects the nearest red blob and returns its egocentric position
def detectOneBlob(clientId, youBotCam, homoMatrix):
    print("Start finding one red blob")
    blobList = []

    err, res, image = vrep.simxGetVisionSensorImage(clientId, youBotCam, 0, vrep.simx_opmode_buffer)

    if err == vrep.simx_return_ok:
        # do some image stuff ----------------------------------------------------------------------------------

        cv2Image = colorDet.convertToCv2Format(image, res)

        tempBlobs = getRedBlobs(cv2Image, homoMatrix, clientId)

        count = 0
        for tb in tempBlobs:
            count = 0
            for b in blobList:
                if isSameBlob(tb[0], b[0]) and tb[1] == b[1]:
                    count = count + 1

            if count == 0:
                print("Added", tb, " to blobList")
                blobList.append(tb)

    # get the closest blob
    currentPos = move.getPos(clientId)[0]
    closestBlob = blobList[0]
    closestDistance = move.getDistanceBetweenPoints(blobList[0], currentPos)
    for blob in blobList:
        distNewBlob = move.getDistanceBetweenPoints(blob[0], currentPos)# distance to new blob
        if(closestDistance-distNewBlob>0): # check if the current blobs position is nearer than the current closest blob
            # set new blob as nearest blob and update closest distance
            closestBlob = blob
            closestDistance = distNewBlob

    print("End find one red blob")
    closestBlob[0] = colorDet.globalToEgocentric(closestBlob[9], clientId)
    return closestBlob[0]


# returns red blobs in a picture
def getRedBlobs(img, homoMatrix, clientID):
    imgCopy = img
    iC = img
    points = []
    boundariesRed = ([0, 0, 190], [86, 86, 255])

    img = imgCopy
    cnts = colorDet.getContours(img, boundariesRed)

    for k in cnts:
        newPoint = np.dot(homoMatrix, colorDet.getBottom(k))
        newPoint = newPoint / newPoint[2]
        newPoint = colorDet.egocentricToGlobal(newPoint, clientID)
        # print(newPoint)

        points.append((newPoint, boundariesRed))

    #cv2.drawContours(iC, cnts, -1, (255, 255, 255), 1)
    # cv2.imshow("Current Image of youBot", iC)
    # cv2.waitKey(0)

    return points

# returns red blobs in a picture with its picture coordinate
def getRedBlobPicture(img):
    points = []
    boundariesRed = ([0, 0, 190], [86, 86, 255])

    contours = colorDet.getContours(img, boundariesRed)

    for k in contours:
        newPoint = colorDet.getBottom(k)

        points.append(newPoint)

    return points

'''
def getListFromQueue(queue):
    tempQueue = Queue()
    list = []
    while not queue.empty():
        object = visitedBlobsQueue.get()
        tempQueue.put(object)
        list.append(object)
    print("List from Queue made: ",tempQueue, " and ",list)
    return list, tempQueue
'''

'''
Arm Kinematics functions
'''

# grabs a blob and returns the next state, also removes the grabed blob from the blobsList
def grabBlob(clientID, blobsList):
    print("Start grab blob")
    blobToGrab = blobsList[0]
    # TODO: Implement grabing and specify state
    print("Stop grab blob")
    return 6, blobsList[1:]

# This function compares two points with some latitude
def isSameBlob(pointA, pointB):
    latituteRadius = 1.3
    if move.getDistanceBetweenPoints(pointA, pointB) <= latituteRadius:
        return True
    return False

def alignToBlock(clientID):
    # TODO rotate by 90°
    # TODO use PD to move right to the block
    return 4        # go to the grab state