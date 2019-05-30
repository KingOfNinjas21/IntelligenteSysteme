from queue import Queue
import bugFunctions as bug
import movementFunctions as move
import vrep
import time
import rangeSensorFunctions as rangeSensor
import math


def initPath():
    queue = Queue()
    explorePath = [(3.0, 1.0), (4.4, 4.0), (-4.2, 5.5), (-3.5, -5.0), (5.0, -4.0),(0.0, -0.5)]
    for i in range(len(explorePath)):
        print ("added to queue: ", explorePath[i])
        queue.put(explorePath[i])
    return queue


# Implementation of distbug algorithm
def distB(clientID, sensorHandles, targetPos):

    isNotGoal = True
    while (isNotGoal):
        isNotGoal , hitRay = headTowardsModel(clientID, targetPos, sensorHandles)

        # if headTowardsModel returned False it means it successfully got to the goal point
        if(not isNotGoal):
            print("REACHED GOAL")
            return not isNotGoal

        # if there is no chair and headTowardsModel returned True -> follow the boundary
        else:
            # orient to wall to have a better start when following a wall
            isRight = bug.wallOrient(clientID, sensorHandles, hitRay, False)
            bug.followBoundary(clientID, sensorHandles, isRight)

        print("Bot is in goal: {}".format(not isNotGoal))

    return not isNotGoal


# Get to the modelName model (name of the target)
# returns false if the bot successfully reached the target and true if the bot encountered an obstacle
def headTowardsModel(clientID, targetPos, rangeSensorHandles):
    print("HeadTowardsModel: start")

    # get the needed position information's of youBot and modelName (target)
    xTarget = targetPos[0]
    yTarget = targetPos[1]
    pos, ori = move.getPos(clientID)

    # calculate the target orientation based on the current youBot position
    targetOrientation = bug.calcTargetOrient(clientID, pos[0], pos[1], xTarget, yTarget)

    # rotate towards modelName until target orientation is reached
    move.rotateUntilOrientation(clientID, targetOrientation)

    # calculate the distance between youBot and target
    dist = bug.calcDistanceToTarget(pos[0], pos[1], xTarget, yTarget)

    #dirve for dist meters or stop when encountering an obstacle
    case, hit = forwardUntilObstacleAnywhere(targetPos, targetOrientation, clientID, rangeSensorHandles)

    return case, hit

# drives forwards for meter meters as long as there is no obstacle around the bot within the distance of 0.3
# around the bot means ca +90 and -90 degrees(from the front)
# returns true if the bot encounterd an obstacle, and false if the bot drove for meter meters
def forwardUntilObstacleAnywhere(targetPos,targetOrientation , clientID, rangeSensorHandles):
    # set velocety to 0
    move.setWheelVelocity(clientID, 0)
    # start moving
    move.startMoving(clientID)
    # continuously check traveled distance
    distance = 0.0
    dt = 0.0
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
        xCurrent, yCurrent = move.getPos(clientID)[0][:-1]
        #x, y, w = move.odometry(x, y, 0.0, move.FORWARD_VEL, 0.0, 0.0, dt)
        distance = move.getDistanceBetweenPoints([x,y], [xCurrent, yCurrent])
        time.sleep(1.0e-06)         # problems with very small time slices -> little delay (if you have a bad angle calculation on your pc try to change this value)
        end = time.time()


        if abs(move.getOrientation(clientID)-move.calcTargetOrient(xCurrent, yCurrent, targetPos[0], targetPos[1]))>3:
            move.rotateUntilOrientation(clientID, targetOrientation)
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