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


armPart1 = 155
armPart2 = 135
armPart3 = 171
uBotHight =  287

def initPath():
    exploreQueue = Queue()
    for path in c.explorePaths:
        tempQueue = Queue()
        for i in range(len(path)):
            #print ("added to exploreQueue: ", path[i])
            tempQueue.put(path[i])
        exploreQueue.put(tempQueue)

    basketQueue = Queue()
    for path in c.basketPaths:
        tempQueue = Queue()
        for i in range(len(path)):
            #print ("added to basketQueue: ", path[i])
            tempQueue.put(path[i])
        basketQueue.put(tempQueue)


    orientationQueue = Queue()
    for ori in c.exploreOrientation:
        orientationQueue.put(ori)

    blockColorQueue = Queue()
    for color in c.blockColor:
        blockColorQueue.put(color)

    return exploreQueue, basketQueue, orientationQueue, blockColorQueue



def init_state(youBotCam, clientID):
    print("Current state: initState")
    explorePaths, basketPaths, orientations, blockColors = initPath()
    nextState = 5 # 5 = follow next explore path
    # init H-Matrix
    return nextState, explorePaths, basketPaths, orientations, blockColors, colorDet.get_H_matrix(c.gCX, youBotCam, clientID)



'''
Needed adjusted movement functions 
'''

# follows the next path from the queue basketPath
# if no explore path in queue explorePaths is left, programm comes to an end
def followExplorePath(clientID, sensorHandles, explorePaths, orientations):
    print("Current state: follow next explore path state")
    print("Start following explore path")
    if not explorePaths.empty():
        nextPath = explorePaths.get()
        while not nextPath.empty():
            targetPos = nextPath.get()
            nextState = -1
            isNotGoal = True
            while (isNotGoal):
                isNotGoal, hitRay = headTowardsModel(clientID, targetPos, sensorHandles)

                # if headTowardsModel returned False it means it successfully got to the goal point
                if (not isNotGoal):
                    print("REACHED POINT")

                # youBot encountered an little bot -> headTowardsModel returned True -> wait a few seconds and try again to reach the next position
                else:
                    print("encountered dick head -> i will wait")
                    # wait 5 seconds
                    time.sleep(5)

        # orient to blob
        move.rotateUntilOrientation(clientID, orientations.get())

        nextState = 2  # 2 = detect blob state
        print("End following explore path")
    else:
        nextState = 0  # 0 = finish state
        print("No paths left")

    return nextState

# follows the next path from the queue basketPath
# fails if queue is empty
def followBasketPath(clientID, sensorHandles, basketPaths, blockColors):
    print("Current state: following next basket path")
    print("Start following basket path")
    if not basketPaths.empty():
        nextPath = basketPaths.get()
        while not nextPath.empty():
            targetPos = nextPath.get()
            nextState = -1
            isNotGoal = True
            while (isNotGoal):
                isNotGoal, hitRay = headTowardsModel(clientID, targetPos, sensorHandles)

                # if headTowardsModel returned False it means it successfully got to the goal point
                if (not isNotGoal):
                    print("REACHED POINT")

                # youBot encountered an little bot -> headTowardsModel returned True -> wait a few seconds and try again to reach the next position
                else:
                    print("encountered dick head -> i will wait")
                    # wait 5 seconds
                    time.sleep(5)

        # align to basket to drop the block
        color = blockColors.get()
        # if color = 0 -> red block -> align to red basket
        # if color = 1 -> blue block -> align to blue basket
        x, y, ori = c.basketCoordinate[color]
        move.moveToCoordinate(x, y, clientID)
        move.rotateUntilOrientation(clientID, ori)
        move.sideway(c.sidewardDistToBasket, clientID, False)

        nextState = 8  # 8 = drop blob state
        print("End following basket path")
    else:
        print("Error -> no basket paths left")
        nextState = -1  # -1 = fail state, which shouldn't happen
    return nextState

# Get to the targetPos [x,y]
# returns false if the bot successfully reached the target and true if the bot encountered an obstacle
def headTowardsModel(clientID, targetPos, rangeSensorHandles):
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
        #for i in range(95, 588): #95 and 588 are estimated values for data from range sensor which are between -90 and 90 degrees (0 is front)
        for i in range(75, 602): #95 and 588 are estimated values for data from range sensor which are between -90 and 90 degrees (0 is front)
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
    print("Current state: get to next blob")
    nextBlob = blobsList[0]
    visitedBlobsList.append(nextBlob)

    # get robot pos
    roboPos, ori = move.getPos(clientID)

    # get shorten egocentric coordinates to the blob
    xA = nextBlob[0] - roboPos[0]
    yA = nextBlob[1] - roboPos[1]
    lenA = math.sqrt(xA * xA + yA * yA)

    shortenXA = xA * (1 - (c.maxDistToBlock / lenA))
    shortenYA = yA * (1 - (c.maxDistToBlock / lenA))

    # transform egocentric coordinates to global space
    x = shortenXA + roboPos[0]
    y = shortenYA + roboPos[1]

    # move forward until block dist - constants.maxDistToBlock
    move.moveToCoordinate(x, y, clientID)

    # rotate by 90° to get vision to the blob
    move.rotate(90, clientID, False)

    nextState = 6   # align to blob state
    return nextState, blobsList, visitedBlobsList


# moves youBot back to the posBeforeMoveToBlob point
def moveBack(clientID, posBeforeMoveToBlob):
    print("Start moving back to " + posBeforeMoveToBlob)
    # TODO: move back
    #  maybe use the following for moving back: ex.headTowardsModel(clientID, posBeforeMoveToBlob, hokuyo)

    state = 3  # 3 = move to next blob
    print("End moving back")
    return state


# drives the youBot to a certain position, specified by goal ( [x,y] coordinate )
def pdControl(clientID, youBotCam, goal):
    print("begin PD control")

    wheelJoints = move.getWheelJoints(clientID)
    err, res, image = vrep.simxGetVisionSensorImage(clientID, youBotCam, 0, vrep.simx_opmode_buffer)
    cv2Image = colorDet.convertToCv2Format(image, res)

    #cv2.imshow("Current Image of youBot", cv2Image)
    #cv2.waitKey(0)

    # initialize variables for while loop
    cor = getRedBlobPicture(cv2Image)
    preCor = cor
    rotVel = 0.0
    forwBackVel = 999  # just a random number to initialize while loop
    leftRightVel = 999  # just a random number to initialize while loop
    while not velOk(forwBackVel, leftRightVel):
        # retrieve new coordinate of red blob
        err, res, image = vrep.simxGetVisionSensorImage(clientID, youBotCam, 0, vrep.simx_opmode_buffer)
        cv2Image = colorDet.convertToCv2Format(image, res)
        cor = getRedBlobPicture(cv2Image)

        # calculate new velocities
        forwBackVel = 0.1 * (cor[0][0] - goal[0]) - 0.002 * (cor[0][0] - preCor[0][0])
        leftRightVel = 0.08 * (cor[0][1] - goal[1]) - 0.001 * (cor[0][1] - preCor[0][1])

        # save current coordinates as previous coordinates
        preCor = cor

        # update velocities
        move.setWheelVelocity(clientID, 0.0)

        vrep.simxPauseCommunication(clientID, True)
        for i in range(0, 4):
            vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], move.wheelVel(forwBackVel/10.0, leftRightVel/10.0, rotVel)[i],
                                            vrep.simx_opmode_oneshot)
        vrep.simxPauseCommunication(clientID, False)

    # stop moving
    move.setWheelVelocity(clientID, 0.0)

    #cv2.imshow("Current Image of youBot", cv2Image)
    #cv2.waitKey(0)

    print("end PD control")


def velOk(forwBackVel, leftRightVel):
    tolerance = 0.1
    forwBackOk = forwBackVel < tolerance and forwBackVel > -tolerance
    leftRightOk = leftRightVel < tolerance and leftRightVel > -tolerance
    return forwBackOk and leftRightOk


def alignToBlob(clientID,youBotCam):
    print("Current state: Align to blob")
    nextState = 4  # 4 = grab state

    # TODO rotate by 90°

    # use PD to move right to the block
    pdControl(clientID, youBotCam, c.grabPosition)
    print("End aligning to blob")
    return nextState


'''
Color Detection Functions
'''

# finds all new blobs, that are not already visited
def findBlobs(clientID, youBotCam, H, currentBlobsList, visitedBlobsList):
    print("Current state: find blobs state")
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
    blobList = []

    err, res, image = vrep.simxGetVisionSensorImage(clientId, youBotCam, 0, vrep.simx_opmode_buffer)

    if err == vrep.simx_return_ok:
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
    print("End find all blobs")
    return blobList

# detects the nearest red or blue blob and returns its egocentric position
def detectOneBlob(clientId, youBotCam, homoMatrix):
    print("Start finding one red blob")
    blobList = []

    err, res, image = vrep.simxGetVisionSensorImage(clientId, youBotCam, 0, vrep.simx_opmode_buffer)

    if err == vrep.simx_return_ok:
        # do some image stuff ----------------------------------------------------------------------------------

        cv2Image = colorDet.convertToCv2Format(image, res)

        tempBlobs = getRedBlueBlobs(cv2Image, homoMatrix, clientId)

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
    closestDistance = move.getDistanceBetweenPoints(blobList[0][0], currentPos)
    for blob in blobList:
        distNewBlob = move.getDistanceBetweenPoints(blob[0], currentPos)# distance to new blob
        if(closestDistance-distNewBlob>0): # check if the current blobs position is nearer than the current closest blob
            # set new blob as nearest blob and update closest distance
            closestBlob = blob
            closestDistance = distNewBlob
    print("COORD: ", closestBlob[0])
    print("End find red or blue blob")
    returnCorrd = colorDet.globalToEgocentric(closestBlob[0], clientId)
    return returnCorrd


# returns red blobs in a picture
def getRedBlueBlobs(img, homoMatrix, clientID):
    imgCopy = img
    iC = img
    points = []
    boundariesRed = ([0, 0, 190], [86, 86, 255])
    boundariesBlue = ([190, 0, 0], [255, 85, 85])
    colors = [boundariesRed, boundariesBlue]
    img = imgCopy

    for c in colors:
        img = imgCopy
        cnts = colorDet.getContours(img, c)
        for k in cnts:
            if c == boundariesRed:
                print("Found red blob -> moving to basket for red blobs")
            else:
                print("Found blue blob -> moving to basket for blue blobs")
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

# This function compares two points with some latitude
def isSameBlob(pointA, pointB):
    latituteRadius = 1.3
    if move.getDistanceBetweenPoints(pointA, pointB) <= latituteRadius:
        return True
    return False


'''
Arm Kinematics functions
'''

# grabs a blob and returns the next state, also removes the grabed blob from the blobsList
def grabBlob(clientID, h_matrix, youBotCam):
    print("Current state: grab blob")
    print("Start grab blob")
    nextState = 7

    # get position of next blob
    blobPos = detectOneBlob(clientID, youBotCam, h_matrix)
    blobPos = colorDet.egocentricToGlobal(blobPos, clientID)
    pos, orient = getArmPos(clientID)

    distance = 1000*move.calcDistanceToTarget(pos[0],pos[1],blobPos[0],blobPos[1])
    a,b,y = armPos(120, distance)
    x = getAngle(clientID, blobPos[0], blobPos[1])
    moveArm(clientID,x ,20,70,0,0)
    moveArm(clientID,x,a, 45,45,0)
    #moveArm(clientID, x,a , b, 45,0)
    moveArm(clientID, x,a, b, y, 0)
    closeHand(clientID)


    print("Stop grab blob")

    #Save Pos
    moveArm(clientID, -0, 20,70,0,0)
    return nextState

def moveArm(clientID, bottomAngle, a,b,y, handAngle):
    #retrieve arm joints
  armJointsHandle=np.empty(5, dtype=np.int); armJointsHandle.fill(-1)
  for i in range(0, 5):
      res,armJointsHandle[i]=vrep.simxGetObjectHandle(clientID,'youBotArmJoint%d'%i,vrep.simx_opmode_oneshot_wait)
 
  #define grasp joints
  # ...
  graspJoints = [bottomAngle*math.pi/180, a*math.pi/180, b*math.pi/180, y*math.pi/180, handAngle*math.pi/180];
  #move arm
  vrep.simxPauseCommunication(clientID, True)
  for i in range(0, 5):
      res = vrep.simxSetJointTargetPosition(clientID, armJointsHandle[i], graspJoints[i],vrep.simx_opmode_oneshot);
  vrep.simxPauseCommunication(clientID, False)
  time.sleep(2)

def openHand(clientID):
    #open the gripper
    res = vrep.simxSetIntegerSignal(clientID, 'gripper_open',1,vrep.simx_opmode_oneshot_wait)
def closeHand(clientID): 
    #close the gripper
    res = vrep.simxSetIntegerSignal(clientID, 'gripper_open',0,vrep.simx_opmode_oneshot_wait)
    time.sleep(2)


def getArmPos(clientID):
    res, base = vrep.simxGetObjectHandle(clientID, 'ME_Arm1_m0_sub0_sub0', vrep.simx_opmode_oneshot_wait)
    base_pos = vrep.simxGetObjectPosition(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
    base_orient = vrep.simxGetObjectOrientation(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
    return base_pos[1], base_orient[1]


def getBlobPos(clientID):
    res, base = vrep.simxGetObjectHandle(clientID, 'redCylinder4', vrep.simx_opmode_oneshot_wait)
    base_pos = vrep.simxGetObjectPosition(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
    base_orient = vrep.simxGetObjectOrientation(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
    return base_pos[1], base_orient[1]

def getAngle(clientID, targetX, targetY):
    
    pos, orient = getArmPos(clientID)
    blobPos, blobOrient = getBlobPos(clientID) #TODO blob coords from sensor 
    uPos, uOrient = move.getPos(clientID)

    #for testing
    targetX = blobPos[0]
    targetY = blobPos[1]

    print(move.calcTargetOrient(pos[0],pos[1],targetX, targetY)) 
    orient[2] =  orient[2] - math.pi
    print(180/math.pi*orient[2])
    a = move.calcTargetOrient(pos[0],pos[1],targetX, targetY) - 180/math.pi*orient[2]
    print(a)

    return a 
  
def armPos(height, dist):
        height = height - uBotHight+armPart3*np.sin(math.pi/180*30)  #because ubot arm is on top of the robot (-armPart3 if arm 90 degree to the ground)
        print("height: " , height)
        dist = dist - armPart3*np.cos(math.pi/180*30) #lastPart of the arm is parallel to the ground (+armPart3 if arm 90 degree to the ground)
        print(dist)
        E2 = height * height + dist *dist #diagonaly between the 2 joint and the armcenter on the robot 
        print(E2)
        if(math.sqrt(E2) > armPart2+armPart1): #arm cant reach the blob
            return 0,0,0
        b1 = np.arccos((E2 - armPart1*armPart1 - armPart2*armPart2)/(-2*armPart1*armPart2)) #angle between the 1 and 2 armPart
        print(b1)
        b = math.pi - b1 #angle for the second joint


        if(E2 != 0):
            a1 =np.arccos((armPart2*armPart2-E2 - armPart1*armPart1)/(-2*math.sqrt(E2)*armPart1)) #angle between arm and E

        a2 = np.arccos(dist/math.sqrt(E2)) #angle between E and de ground 
        if(height > 0):
            a = math.pi/2 - a1 - a2 # angle for the first joint
        if(height < 0): 
            a = math.pi/2 + (a2 - a1)  # angle for the first joint
        
        y = math.pi/2 - a - b + math.pi/180*30 #angle for the third (math.pi if last armPart 90 degree to the ground, math.pi/2 if parallel)

       # print(180/math.pi * a," ",   180/math.pi*b," ",180/ math.pi*y)

        return 180/math.pi * a,180/math.pi*b,180/ math.pi*y

def dropBlob(clientID):
    print("Current state: drop blob state")
    print("Start drop")
    a = getAngle(clientID)
    nextState = 5
    moveArm(clientID, a, 20,70,0,0)

    openHand(clientID)

    #Save Pos
    moveArm(clientID, 0, 20 , 70, 0)
    print("End drop")
    return nextState