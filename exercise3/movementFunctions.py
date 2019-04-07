import vrep
import numpy as np
import time
import math
import rangeSensorFunctions as rangeSensor


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
BASIC FUNCTIONS FOR RETRIEVING INFORMATION'S
************************************
"""

# get the 4 wheel joints of the youBot
def getWheelJoints(clientID):
    # Retrieve wheel joint handles:
    wheelJoints = np.empty(4, dtype=np.int)
    wheelJoints.fill(-1)  # front left, rear left, rear right, front right
    res, wheelJoints[0] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep.simx_opmode_oneshot_wait)
    res, wheelJoints[1] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_oneshot_wait)
    res, wheelJoints[2] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep.simx_opmode_oneshot_wait)
    res, wheelJoints[3] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_oneshot_wait)

    return wheelJoints

def printPos(clientID):
    res, base = vrep.simxGetObjectHandle(clientID, 'youBot_center', vrep.simx_opmode_oneshot_wait)
    base_pos = vrep.simxGetObjectPosition(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
    base_orient = vrep.simxGetObjectOrientation(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
    vrep.simxGetPingTime(clientID)  # make sure that all streaming data has reached the client at least once

    print ("Position: ", base_pos[1])
    print ("Orientation: alpha: {}, beta: {}, gamma: {}".format(base_orient[1][0]*180.0/math.pi, base_orient[1][1]*180.0/math.pi, base_orient[1][2]*180.0/math.pi))


def getPos(clientID):
    res, base = vrep.simxGetObjectHandle(clientID, 'youBot_center', vrep.simx_opmode_oneshot_wait)
    base_pos = vrep.simxGetObjectPosition(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
    base_orient = vrep.simxGetObjectOrientation(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
    return base_pos[1], base_orient[1]

# returns the youbot orientation in degree
def getOrientation(clientID):
    res, base = vrep.simxGetObjectHandle(clientID, 'youBot_center', vrep.simx_opmode_oneshot_wait)
    base_orient = vrep.simxGetObjectOrientation(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
    return base_orient[1][2]*180.0/math.pi

# This function substracts two orientation. It returns the substracted orientation in a range (-180,+180].
def substractOrientation(orientationA, orientationB):
    newOrientation = orientationA - orientationB

    if newOrientation <= -180:               # new orientation changed from negative to positive
        newOrientation = 180 + (newOrientation + 180)
    elif newOrientation > 180:              # new orientation changed from positive to negative
        newOrientation = -180 + (newOrientation - 180)

    return newOrientation

def wheelVel(forwBackVel, leftRightVel, rotVel):
    return np.array([-forwBackVel-leftRightVel-rotVel, -forwBackVel+leftRightVel-rotVel, -forwBackVel+leftRightVel+rotVel, -forwBackVel-leftRightVel+rotVel])


# this function calculates the new position of the robot based on the old position and the speed of every wheel and the time difference
def odometry(x, y, w, forwBackVel, leftRightVel, rotVel, dt):
    vf, vr, w0 = formatVel(forwBackVel, leftRightVel, rotVel)

    o = w0 * dt
    dx = -vf * dt * math.sin(o) + vr * dt * math.cos(o)
    dy = vf * dt * math.cos(o) + vr * dt * math.sin(o)

    x = x + dx
    y = y + dy
    w = w + o

    return x, y, w


# This function formats the spin of every wheel to a forward-, leftRightvelocety and an angle
def formatVel(forwBackVel, leftRightVel, rotVel):
    vf = forwBackVel * (B/2.0)
    vr = leftRightVel * (B/2.0)
    w0 = (2.0/(C+D)) * rotVel*(180/math.pi) * (B/2.0)       # format rotVel (radiant) to degree

    return vf, vr, w0


"""
************************************
FORWARD/SIDEWAYS FUNCTIONS
************************************
"""


# drive forward for dist meters
def forward(dist, clientID):
    # set velocety to 0
    wheelJoints = getWheelJoints(clientID)
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], 0, vrep.simx_opmode_oneshot)

    # start moving
    vrep.simxPauseCommunication(clientID, True)
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], wheelVel(FORWARD_VEL, 0.0, 0.0)[i],
                                        vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID, False)

    # continuously check traveled distance
    distance = 0.0
    dt = 0.0
    x = 0.0
    y = 0.0
    while distance <= dist:
        start = time.time()
        x, y, w = odometry(x, y, 0.0, FORWARD_VEL, 0.0, 0.0, dt)
        distance = math.sqrt(x*x+y*y)
        time.sleep(1.0e-06)         # problems with very small time slices -> little delay (if you have a bad angle calculation on your pc try to change this value)
        end = time.time()
        dt = end-start

    # stop moving
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], 0, vrep.simx_opmode_oneshot)

# drive sideways for dist meters
def sideway(dist, clientID, toRight):
    # set velocety to 0
    wheelJoints = getWheelJoints(clientID)
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], 0, vrep.simx_opmode_oneshot)

    # start moving
    vrep.simxPauseCommunication(clientID, True)
    if(toRight):
        for i in range(0, 4):
            vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], wheelVel( 0.0,FORWARD_VEL, 0.0)[i],vrep.simx_opmode_oneshot)
    else: 
        for i in range(0, 4):
            vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], wheelVel( 0.0,-FORWARD_VEL, 0.0)[i],vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID, False)

    # continuously check traveled distance
    distance = 0.0
    dt = 0.0
    x = 0.0
    y = 0.0
    while distance <= dist:
        start = time.time()
        x, y, w = odometry(x, y, 0.0, FORWARD_VEL, 0.0, 0.0, dt)
        distance = math.sqrt(x*x+y*y)
        time.sleep(1.0e-06)         # problems with very small time slices -> little delay (if you have a bad angle calculation on your pc try to change this value)
        end = time.time()
        dt = end-start

    # stop moving
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], 0, vrep.simx_opmode_oneshot)


def setWheelVelocity(clientID, velocity):
    wheelJoints = getWheelJoints(clientID)
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], velocity, vrep.simx_opmode_oneshot)

def startMoving(clientID):
    wheelJoints = getWheelJoints(clientID)
    vrep.simxPauseCommunication(clientID, True)
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], wheelVel(FORWARD_VEL, 0.0, 0.0)[i],
                                        vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID, False)

# drives forwards for meter meters as long as there is no obstacle around the bot within the distance of 0.3
# around the bot means ca +90 and -90 degrees(from the front)
# returns true if the bot encounterd an obstacle, and false if the bot drove for meter meters
def forwardUntilObstacleAnywhere(meter, clientID, rangeSensorHandles):
    # set velocety to 0
    setWheelVelocity(clientID, 0)
    # start moving
    startMoving(clientID)
    # continuously check traveled distance
    distance = 0.0
    dt = 0.0
    x = 0.0
    y = 0.0
    stop = False
    hit = 0 
    while distance <= meter and stop!=True:
        start = time.time()
        # get range sensor data as list of x,y,z,distance
        rangeData = rangeSensor.getSensorData(clientID, rangeSensorHandles)
        # range sensor angle is estimated for about 250 degrees
        for i in range(95, 588): #95 and 588 are estimated values for data from range sensor which are between -90 and 90 degrees (0 is front)
            if rangeData[i][3]<=0.3:
                stop=True
                hit = i
                break

        x, y, w = odometry(x, y, 0.0, FORWARD_VEL, 0.0, 0.0, dt)
        distance = math.sqrt(x*x+y*y)
        time.sleep(1.0e-06)         # problems with very small time slices -> little delay (if you have a bad angle calculation on your pc try to change this value)
        end = time.time()
        dt = end-start

    # stop moving
    setWheelVelocity(clientID, 0)

    return(stop, hit)

"""
************************************
ROTATING FUNCTIONS
************************************
"""


# rotate degree degrees to the right if rotRight is True, otherwise (rotRight=False) rotate to the left
def rotate(degree, clientID, rotRight):
    rotationVel = ROTATE_VEL
    if not rotRight:
        rotationVel *= -1

    # set velocety to 0
    wheelJoints = getWheelJoints(clientID)
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], 0, vrep.simx_opmode_oneshot)

    # start moving
    vrep.simxPauseCommunication(clientID, True)
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], wheelVel(0.0, 0.0, rotationVel)[i],
                                        vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID, False)

    # continuously check traveled distance
    w = 0.0
    dt = 0.0
    while abs(w) <= degree:
        start = time.time()
        x, y, w = odometry(0.0, 0.0, w, 0.0, 0.0, rotationVel, dt)
        time.sleep(1.0e-06)         # problems with very small time slices -> little delay (if you have a bad angle calculation on your pc try to change this value)
        end = time.time()
        dt = end - start

    # stop moving
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], 0, vrep.simx_opmode_oneshot)
    return

# rotate degree degrees to the right if rotRight is True, otherwise (rotRight=False) rotate to the left
# this function uses the system orientation
def rotateSys(degree, clientID, isRotRight):
    if not isRotRight:
        degree *= -1

    startOrient = getOrientation(clientID)
    goalOrient = substractOrientation(startOrient, -degree)

    # start rotation
    rotateUntilOrientation(clientID, goalOrient)

def rotateUntilOrientation(clientID, targetOrient):
    rotationVel = ROTATE_VEL
    wheelJoints = getWheelJoints(clientID)
    currentOrient = getOrientation(clientID)
    safty = False
    print("Calced orientations for rotation: Current Orientation = {} , Target Orientation = {}" .format(currentOrient, targetOrient))

    if(currentOrient > 0 and targetOrient < 0 and abs(targetOrient - currentOrient)> 180):
        rotationVel *= -1
        print("sjffbj")
        startRotating(clientID, rotationVel)
        if(targetOrient + 1) > 180:
            safty = True
        while not (targetOrient + 5.0 > currentOrient and currentOrient > targetOrient -5.0): #fails at 180/-180
            time.sleep(0.05)
            currentOrient = getOrientation(clientID)
            if(currentOrient < -175 and safty == True):
                break

    elif(currentOrient > 0 and targetOrient < currentOrient):
        rotationVel *= 1
        startRotating(clientID, rotationVel)
        if(targetOrient - 1) < -180:
            safty = True
        while targetOrient < currentOrient: #fails at 0
            time.sleep(0.05)
            currentOrient = getOrientation(clientID)
            if(currentOrient > 175) and safty == True:
                break

    elif(currentOrient > 0 and targetOrient > currentOrient):
        rotationVel *= -1
        startRotating(clientID, rotationVel)
        if(targetOrient + 1) > 180:
            safty = True
        while targetOrient > currentOrient: #fails at 180/-180
            time.sleep(0.05)
            currentOrient = getOrientation(clientID)
            if(currentOrient < -175 and safty == True):
                break


    elif(currentOrient < 0 and targetOrient > 0 and abs(targetOrient-currentOrient)>180):
        rotationVel *= 1
        startRotating(clientID, rotationVel)
        if(targetOrient - 1) < -180:
            safty = True
        while not (targetOrient + 5.0 > currentOrient and targetOrient - 5.0 < currentOrient): #fails at 0
            time.sleep(0.05)
            currentOrient = getOrientation(clientID)
            if(currentOrient > 175) and safty == True:
                break
    elif(currentOrient < 0 and targetOrient < currentOrient):
        rotationVel *= 1
        startRotating(clientID, rotationVel)
        if(targetOrient - 1) < -180:
            safty = True
        while targetOrient < currentOrient: #fails at 0
            time.sleep(0.05)
            currentOrient = getOrientation(clientID)
            if(currentOrient > 175) and safty == True:
                break

    else:
        rotationVel *= -1
        startRotating(clientID, rotationVel)
        if(targetOrient + 1) > 180:
            safty = True
        while targetOrient > currentOrient: #fails at 180/-180
            time.sleep(0.05)
            currentOrient = getOrientation(clientID)
            if(currentOrient < -175 and safty == True):
                break



    # stop moving
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], 0, vrep.simx_opmode_oneshot)
    return

def startRotating(clientID, rotationVel):
    # set velocety to 0
    wheelJoints = getWheelJoints(clientID)
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], 0, vrep.simx_opmode_oneshot)

    # start moving
    vrep.simxPauseCommunication(clientID, True)
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], wheelVel(0.0, 0.0, rotationVel)[i],
                                        vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID, False)


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
    pos, ori = getPos(clientID)

    # print where the youBot is and where the target
    print("youBot: x = {}, y = {}".format(pos[0], pos[1]))
    print ("{}: x = {}, y = {}".format(modelName, xTarget, yTarget))

    # calculate the target orientation based on the current youBot position
    targetOrientation = calcTargetOrient(clientID, pos[0], pos[1], xTarget, yTarget)

    # rotate towards modelName until target orientation is reached
    rotateUntilOrientation(clientID, targetOrientation)

    # calculate the distance between youBot and target
    dist = calcDistanceToTarget(pos[0], pos[1], xTarget, yTarget)

    #dirve for dist meters or stop when encountering an obstacle
    case, hit  = forwardUntilObstacleAnywhere(dist, clientID, rangeSensorHandles)

    pos, ori = getPos(clientID) # get new position of bot for the output
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
    botOrient = getOrientation(clientID)

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
            rotateUntilOrientation(clientID, a1)
            isRight= True

        else:
            rotateUntilOrientation(clientID, a2)
            isRight= False

    else:
        print("Turn because of a corner")
        # Determine where the nearest wall on the left and right side is, to know in which direction to turn
        if(rangeData[LEFT_RAY_NINETY][3]<2):
            rotate(90.0, clientID, True)
            isRight = True
        elif(rangeData[RIGHT_RAY_NINETY][3]<2):
            rotate(90.0, clientID, False)
            isRight = False
        wallOrient(clientID, rangeSensorHandles, rayHit, False) # make wall orient after going around corner

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

    pos, ori = getPos(clientID)
    
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
    robotOrientation = getOrientation(clientID)

    angleToTarget = substractOrientation(targetOrientation, robotOrientation)


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
    pos, ori = getPos(clientID)

    # calculate the target orientation
    targetOrientation = calcTargetOrient(clientID, pos[0], pos[1], xTarget, yTarget)

    return targetOrientation

# sequence of movement function calls to get around a chair
def driveAroundChair(clientID):
    rotate(90.0, clientID, True)
    forward(1.0, clientID)
    rotate(90.0, clientID, False)
    forward(1.0, clientID)

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