import vrep
import numpy as np
import time
import math
import rangeSensorFunctions as rangeSensor
import movementFunctions as move


"""
************************************
GLOBAL VARIABLES FOR THE YOUBOT
************************************
"""
# B,C,D in meters
B = 0.1
C = 0.471
D = 0.30046

FORWARD_VEL =  -2.0
SIDEWARD_VEL =  2.0
ROTATE_VEL =  2.0

STEP = 3.5	# constant for the leaving condition
RAY_ANGLE = 120.0/342.0

FRONT_SEN_START = 322
FRONT_SEN_END = 357

LEFT_RAY_NINETY = 603
RIGHT_RAY_NINETY = 82

"""
************************************
BUG-ALGORITHMS FUNCTIONS
************************************
"""

# Get to the modelName model (name of the target)
# returns false if the bot successfully reached the target and true if the bot encountered an obstacle
def headTowardsModel(clientID, modelName, rangeSensorHandles):
    print("HeadTowardsModel: start")

    # get the needed position information's of youBot and modelName (target)
    res, objHandle = vrep.simxGetObjectHandle(clientID, modelName, vrep.simx_opmode_oneshot_wait)
    targetPosition = vrep.simxGetObjectPosition(clientID, objHandle, -1, vrep.simx_opmode_oneshot_wait)
    xTarget = targetPosition[1][0]
    yTarget = targetPosition[1][1]
    pos, ori = move.getPos(clientID)

    # print where the youBot is and where the target
    print("youBot: x = {}, y = {}".format(pos[0], pos[1]))
    print ("{}: x = {}, y = {}".format(modelName, xTarget, yTarget))

    # calculate the target orientation based on the current youBot position
    targetOrientation = calcTargetOrient(clientID, pos[0], pos[1], xTarget, yTarget)

    # rotate towards modelName until target orientation is reached
    move.rotateUntilOrientation(clientID, targetOrientation)

    # calculate the distance between youBot and target
    dist = calcDistanceToTarget(pos[0], pos[1], xTarget, yTarget)

    #dirve for dist meters or stop when encountering an obstacle
    case, hit  = move.forwardUntilObstacleAnywhere(dist, clientID, rangeSensorHandles)

    pos, ori = move.getPos(clientID) # get new position of bot for the output
    print("Current position of bot after headTowardsModel; x = {} , y = {}".format(pos[0], pos[1]))
    print("HeadTowardsModel: end, Finished? {}".format(not case))

    return case, hit

# calculates distance between 2 x,y coordinates
def calcDistanceToTarget(xStart, yStart, xEnd, yEnd):
    distanceToTarget = math.sqrt((xEnd-xStart)*(xEnd-xStart)+(yEnd-yStart)*(yEnd-yStart))
    return distanceToTarget

# returns the orientation from the start points towards the end points based on the v-rep x,y,z axis / orientations
def calcTargetOrient(clientID, xStart, yStart, xEnd, yEnd):
    # target orientation is calculated with the atan(), so we need the opposite side length (GK) and
    # adjacent side (AK)
    GK = abs(float(yEnd-yStart))
    AK = abs(float(xEnd-xStart))

    angle= abs(math.atan2(GK, AK) * 180.0 / math.pi)

    # 4 cases where the target can be based on the current position (xStart and yStart), every case
    # represents one of the 4 quadrants in the x,y,z space
    # Based on the quadrant you can calculate what orientation (0 degree: -y, 180/-180 degree y, 90 degree x,
    # -90 degree -x) is needed to get from start to end
    if xEnd<xStart:

        if yEnd<yStart:
            targetOrient = - 90.0 + angle
        if yEnd>yStart:
            targetOrient = - 90.0 - angle

    elif xEnd>xStart:

        if yEnd < yStart:
            targetOrient = 90.0 - angle
        if yEnd > yStart:
            targetOrient = 90.0 + angle

    else:
        targetOrient = 0

    return targetOrient

# Aligns the robot parallel to the nearest wall: isInOrientState = False
# Second task if isInOrientState = True: Allign the bot parallel to wall in front of the youBot
# The second task is used when encountering an obstacle infront of the bot when following a wall -> Dealing with corners
def wallOrient(clientID, rangeSensorHandles, rayHit, isInOrientState):
    print("WallOrient start")

    # get sensor data from range sensors and the current bot orientation
    rangeData = rangeSensor.getSensorData(clientID, rangeSensorHandles)
    botOrient = move.getOrientation(clientID)

    if(not isInOrientState):
        print("Orient to nearest wall")


        # to know how to align to the nearest wall on the side we
        # use 2 rays to calculate the orientation of the wall -> x1, y2, and x2, y2

        # start calc index of x2, y2 in rangeData
        indexOfSndOrientPoint = rayHit + 10
        for i in range(rayHit + 10, rayHit + 50):
            if ((rangeData[i][3] - rangeData[rayHit][3]) < 0.5):
                indexOfSndOrientPoint = i
                break
        # end calc index

        x1 = rangeData[rayHit][0]
        y1 = rangeData[rayHit][1]
        x2 = rangeData[indexOfSndOrientPoint][0]
        y2 = rangeData[indexOfSndOrientPoint][1]

        # calculate the 2 possiblies of the 2 rays used so we can take the shorter ray, which
        # will be our wanted target orientation
        a1 = calcTargetOrient(clientID, x2, y2, x1, y1)
        a2 = calcTargetOrient(clientID, x1, y1, x2, y2)

        if abs(botOrient-a1)<abs(botOrient-a2):
            move.rotateUntilOrientation(clientID, a1)
            isRight= True

        else:
            move.rotateUntilOrientation(clientID, a2)
            isRight= False

    else:
        print("Turn because of a corner")
        # Determine where the nearest wall on the left and right side is, to know in which direction to turn
        if(rangeData[LEFT_RAY_NINETY][3]<2):
            move.rotate(90.0, clientID, True)
            isRight = True
        elif(rangeData[RIGHT_RAY_NINETY][3]<2):
            move.rotate(90.0, clientID, False)
            isRight = False

    print("WallOrient end")
    return isRight

# checks if a corner is encountered
# returns True if a corner is encountered or False if not
def detectCorner(clientID, rangeSensorHandles, rayHit, maxDist):
    # get the current data from the range sensors
    rangeData = rangeSensor.getSensorData(clientID, rangeSensorHandles)

    # a corner is encountered when at least one distance of the rangeData rays exceeds the max distant by 0.5
    for i in range(rayHit-5, rayHit+5):
        if (rangeData[i][3] > maxDist + 0.5):
            print("corner detected!")
            return True

    return False
    
# This function calculates the leaving condition for the DistBug algorithm.
# For the calculation the minimum distance to the target needs to be tracked (it will be updated in this function).
# distNextObstacle - The distance to another obstacle arount the robot (not the obstacle the robot is following) (max value if no obstacle around)
# returns True if the leaving condtion holds, falls otherwise and the new minDist
def calcLeavingConditin(minDist, distNextObstacle, clientID):
    leave = False

    if distNextObstacle == -1.0:      # non trackable distance, normally returned by calcFreespace
        print("Freespace not Trackable")
        return leave, minDist

    # calc current dist. to target
    res, objHandle = vrep.simxGetObjectHandle(clientID, "Goal", vrep.simx_opmode_oneshot_wait)
    targetPosition = vrep.simxGetObjectPosition(clientID, objHandle, -1, vrep.simx_opmode_oneshot_wait)
    xTarget = targetPosition[1][0]
    yTarget = targetPosition[1][1]

    pos, ori = move.getPos(clientID)
    
    dist = calcDistanceToTarget(pos[0], pos[1], xTarget, yTarget)

    if minDist == -1.0:     # init minDist
        minDist = dist
    
    
    # calc leaving condition
    if dist - distNextObstacle <= minDist - STEP:
        leave = True
    
    
    # set minDist if neccesary
    if dist < minDist:
        minDist = dist

    if distNextObstacle > dist:
        leave = True


    return leave, minDist



# calculates the freespace distance to the goal
# returns the freespace distance or -1.0 if the distance is currently not trackable (calculated index too high or low)
def calcFreeSpace(clientID, sensorHandles):
    # get sensor data and the (calculated) ray index of the ray to the target which is used for rangeData
    rangeData = rangeSensor.getSensorData(clientID, sensorHandles)
    ray = getRayToTarget(clientID)

    # non trackable distance if ray index to target (index of range sensor data) is above 633 or below 50
    if ray < 50 or ray > 633:
        # return false value (-1.0 is interpreted as
        return -1.0

    minDistToObstacle = rangeData[ray][3]

    # search for a smaller distance in rangeData in target direction
    for i in range(ray-50, ray+50):
        if rangeData[i][3] < minDistToObstacle:
            minDistToObstacle = rangeData[i][3]

    return minDistToObstacle

# This function returns the index of the ray pointing to the goal
# Some small tests show that this function has an anomaly by +-4 degrees
def getRayToTarget(clientID):
    # calculate angle to target
    targetOrientation = calcOrientationToTarget(clientID, "Goal")
    robotOrientation = move.getOrientation(clientID)

    angleToTarget = move.substractOrientation(targetOrientation, robotOrientation)


    # choose between left and right sensor and
    # calculate ray pointing to target
    if angleToTarget > 0:   # left sensor
        ray = math.fabs(angleToTarget) / RAY_ANGLE      # calculate ray as float
        ray = round(ray)                            # round to int
        ray += 342                                  # add first idx of left sensor
    else:                   # right sensor
        ray = (120 - math.fabs(angleToTarget)) / RAY_ANGLE
        ray = round(ray)

    return int(ray)

# alternative to calcTargetOrient() where the targetName is passed instead of the x,y-coordinates
def calcOrientationToTarget(clientID, targetName):
    # get target position
    res, objHandle = vrep.simxGetObjectHandle(clientID, targetName, vrep.simx_opmode_oneshot_wait)
    targetPosition = vrep.simxGetObjectPosition(clientID, objHandle, -1, vrep.simx_opmode_oneshot_wait)

    xTarget = targetPosition[1][0]
    yTarget = targetPosition[1][1]

    #get the youBot position
    pos, ori = move.getPos(clientID)

    # calculate the target orientation
    targetOrientation = calcTargetOrient(clientID, pos[0], pos[1], xTarget, yTarget)

    return targetOrientation

# sequence of movement function calls to get around a chair
def driveAroundChair(clientID):
    move.rotate(90.0, clientID, True)
    move.forward(1.0, clientID)
    move.rotate(90.0, clientID, False)
    move.forward(1.0, clientID)

# check if a chair is in front of the youBot
def isChairInFront(sensorData):
    startRay = FRONT_SEN_START - 50
    endRay = FRONT_SEN_END + 50
    sum = 0

    # if a chair is in front is determined by the average detected distance of the rays

    for i in range(startRay , endRay):
        sum += sensorData[i][3]

    avg = sum / (endRay - startRay)

    print("Chair detection avg range: {}".format(avg))
    if avg >= 1.5:
        return True
    else:
        return False


#follows a obstacle until a leaving condition is found (for dist bug)
def followBoundary(clientID, sensorHandles, rightSide):
    print("Follow boundary: start")

    # set rayHit to the ray index where the followed wall is
    if rightSide:
        rayHit = LEFT_RAY_NINETY
    else:
        rayHit = RIGHT_RAY_NINETY

    targetDistanceToWall = 0.5
    minRange = targetDistanceToWall-0.05
    maxRange = targetDistanceToWall+0.05
    counter = 0
    minDistToTarget = -1.0

    while True:
        move.startMoving(clientID)
        while not detectCorner(clientID,sensorHandles, rayHit, (minRange+maxRange)/2.0): #not abs(oldDis - newDis)>1
            rangeData = rangeSensor.getSensorData(clientID, sensorHandles)

            for i in range(move.FRONT_SEN_START,move.FRONT_SEN_END):          # check for a pool of front sensors if there is an obstacle
                if rangeData[i][3] <= 0.5:
                    print("FLALALALALAL")
                    wallOrient(clientID, sensorHandles, rayHit, True)
                    break

            move.startMoving(clientID)
            # Only check the following every 5 iterations, so that the correction of following doesn't happen too often
            if counter % 5 == 0:

                rangeToWallNew = rangeData[rayHit][3]
                print("Current range to wall: {}".format(rangeToWallNew))

                # if the distance to the wall is to little drive away from the wall
                if rangeToWallNew<minRange:
                    move.setWheelVelocity(clientID, 0)
                    move.sideway(minRange -rangeToWallNew ,clientID,rightSide)
                    move.startMoving(clientID)
                    wallOrient(clientID, sensorHandles, rayHit, False)

                # if the distance to the wall is too high -> drive towards the wall
                elif rangeToWallNew>maxRange:
                    move.setWheelVelocity(clientID, 0)
                    move.sideway(rangeToWallNew - maxRange ,clientID, not rightSide)
                    move.startMoving(clientID)
                    wallOrient(clientID, sensorHandles, rayHit, False)

            counter+=1

            # check the leaving conditions
            freespace = calcFreeSpace(clientID, sensorHandles)
            leavingCondition, minDistToTarget = calcLeavingConditin(minDistToTarget, freespace, clientID)

            if leavingCondition:
                print("Leaving follow boundary ... cause: leaving condition")
                print("Follow boundary: end")
                return

        print("drive around corner")

        goAroundCorner(clientID, sensorHandles, rightSide, rayHit)

        #check leaving conditions here again after going around a corner
        freespace = calcFreeSpace(clientID, sensorHandles)
        leavingCondition, minDistToTarget = calcLeavingConditin(minDistToTarget, freespace, clientID)

        if leavingCondition:
            print("leaving cause of leaving condition")
            return

# function handle the movement when going around a corner
def goAroundCorner(clientID, sensorHandles, rightSide, rayHit):
    print("Going around corner: start")

    # first drive forward -> rotate -> drive forward again
    move.forward(0.8, clientID)
    move.rotate(90, clientID, not rightSide)
    move.forward(0.8, clientID)

    # check if the bot has to do a uturn because of the corner or he can get back to following the obstacle
    if(normalBorder(clientID, sensorHandles, rightSide)):
        print("just a corner")
    else:
        print("U-Turn")
        while True:
            # when encountering the u-turn option drive forward by 0.2 as long as there is no obstacle which can be followed
            # -> there is a obstacle to follow when more than 5 rays hit a obstacle
            move.forward(0.2, clientID)
            rangeData = rangeSensor.getSensorData(clientID, sensorHandles)
            countRays = 0
            for i in range(rayHit - 80, rayHit + 80):
                if rangeData[i][3] < 0.8:
                    countRays += 1
            if countRays < 5:
                break
        move.rotate(90, clientID, not rightSide)
        move.forward(1.0, clientID)
        wallOrient(clientID, sensorHandles, rayHit, False)
    print("Going around corner: end")

# check if there is a wall in rayHit (index for rangeData) direction
def normalBorder(clientID, sensorHandles, rayHit):
    rangeData = rangeSensor.getSensorData(clientID, sensorHandles)
    for i in range(rayHit-15, rayHit+15):
        for j in range(i+5, i+30):
            # u-turn encountered when distance difference is greater than 0.2 for at least 2 points
            if(abs(rangeData[i][3] - rangeData[j][3]) > 0.2):
                return False
    return True

# Implementation of distbug algorithm
def distB(clientID, sensorHandles, goalName):

    isNotGoal = True
    while (isNotGoal):
        isNotGoal , hitRay = headTowardsModel(clientID, goalName, sensorHandles)

        # if headTowardsModel returned False it means it successfully got to the goal point
        if(not isNotGoal):
            print("FINISHED")
            return

        # if there is a chair in front -> go around chair
        if(isChairInFront(rangeSensor.getSensorData(clientID, sensorHandles))):
            driveAroundChair(clientID)

        # if there is no chair and headTowardsModel returned True -> follow the boundary
        else:
            # orient to wall to have a better start when following a wall
            isRight = wallOrient(clientID, sensorHandles, hitRay, False)
            followBoundary(clientID, sensorHandles, isRight)

        print("Bot is in goal: {}".format( not isNotGoal))



#follows a obstical until it is at the closes point to a goal (for bug1)
def followObstacle(clientID, sensorHandles, rightSide, roundFinished):
    print("Follow boundary: start")

    # set rayHit to the ray index where the followed wall is
    if rightSide:
        rayHit = LEFT_RAY_NINETY
    else:
        rayHit = RIGHT_RAY_NINETY

    targetDistanceToWall = 0.5
    minRange = targetDistanceToWall-0.05
    maxRange = targetDistanceToWall+0.05
    counter = 0

    startPoint = (move.getPos(clientID)[0][0], move.getPos(clientID)[0][1]) #point where ubot first hit the obstacle 
    res, objHandle = vrep.simxGetObjectHandle(clientID, 'Goal', vrep.simx_opmode_oneshot_wait)
    RES, targetPosition = vrep.simxGetObjectPosition(clientID, objHandle, -1, vrep.simx_opmode_oneshot_wait)
    goalPoint = (targetPosition[0], targetPosition[1])
    

    minDist = move.getDistanceBetweenPoints(startPoint, goalPoint) #shortest dist to the goal
    minPoint = startPoint #coordinates of the nearest point to the goal
    disToMin = 0

    while True:
        move.startMoving(clientID)

        #variables for distance tracking
        x = 0.0
        y = 0.0
        dt = 0.0
        while not detectCorner(clientID,sensorHandles, rayHit, (minRange+maxRange)/2.0): #not abs(oldDis - newDis)>1
            start = time.time()                     # start time for distance tracking

            x, y, distanceTravled = move.calcTraveldDistance(x, y, dt)

            rangeData = rangeSensor.getSensorData(clientID, sensorHandles)

            for i in range(move.FRONT_SEN_START,move.FRONT_SEN_END):          # check for a pool of front sensors if there is an obstacle
                if rangeData[i][3] <= 0.5:
                    print("FLALALALALAL")
                    wallOrient(clientID, sensorHandles, rayHit, True)
                    break

            move.startMoving(clientID)
            # Only check the following every 5 iterations, so that the correction of following doesn't happen too often
            if counter % 5 == 0:

                rangeToWallNew = rangeData[rayHit][3]
                print("Current range to wall: {}".format(rangeToWallNew))

                # if the distance to the wall is to little drive away from the wall
                if rangeToWallNew<minRange:
                    move.setWheelVelocity(clientID, 0)
                    move.sideway(minRange -rangeToWallNew ,clientID,rightSide)
                    move.startMoving(clientID)
                    wallOrient(clientID, sensorHandles, rayHit, False)

                # if the distance to the wall is too high -> drive towards the wall
                elif rangeToWallNew>maxRange:
                    move.setWheelVelocity(clientID, 0)
                    move.sideway(rangeToWallNew - maxRange ,clientID, not rightSide)
                    move.startMoving(clientID)
                    wallOrient(clientID, sensorHandles, rayHit, False)

            counter+=1
            currentPoint = (move.getPos(clientID)[0][0], move.getPos(clientID)[0][1])
            if(roundFinished == False and distanceTravled > 1): #checks if the ubot drove around the obstacle (distanceTraveld > 1 is tolleranz to prevent, that the ubot thinks he finished a rond when he is at the starting point for the first time)
                roundFinished = move.isSamePoint(startPoint, currentPoint)


            if(move.getDistanceBetweenPoints(currentPoint, goalPoint) < minDist): #checks for the min distance to the goal 
                minDist = move.getDistanceBetweenPoints(currentPoint, goalPoint)
                minPoint = currentPoint
                disToMin = distanceTravled


            if(roundFinished and move.isSamePoint(minPoint, currentPoint) and 2*disToMin - 1 < distanceTravled): #checks if the robot is again at the nearest point, again with toleranz 
                print("min Point found")
                return

            end = time.time()                                           # end time for distance tracking
            dt = end - start


        print("drive around corner")

        goAroundCorner(clientID, sensorHandles, rightSide, rayHit)

        if(roundFinished and move.isSamePoint(minPoint, currentPoint)):
            print("min Point found")
            return


#impliments bug1 algo 
def bug1(clientID, sensorHandles, goalName):
    isNotGoal = True
    while (isNotGoal):
        isNotGoal , hitRay = headTowardsModel(clientID, goalName, sensorHandles)

        # if headTowardsModel returned False it means it successfully got to the goal point
        if(not isNotGoal):
            print("FINISHED")
            return


        #
        else:
            # orient to wall to have a better start when following a wall
            isRight = wallOrient(clientID, sensorHandles, hitRay, False)
            followObstacle(clientID, sensorHandles, isRight, True)

        print("Bot is in goal: {}".format( not isNotGoal))

