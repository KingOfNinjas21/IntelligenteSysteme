from queue import Queue
import bugFunctions as bug
import movementFunctions as move

def initPath():
    queue = Queue()
    explorePath = [(3.0, 1.0), (4.4, 4.0), (-5.0, 3.0), (-3.5, -5.0), (5.0, -4.0),(0.0, -0.5)]
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

        print("Bot is in goal: {}".format( not isNotGoal))
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
    case, hit  = move.forwardUntilObstacleAnywhere(dist, clientID, rangeSensorHandles)

    pos, ori = move.getPos(clientID) # get new position of bot for the output

    return case, hit