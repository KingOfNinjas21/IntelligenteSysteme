import vrep
import numpy as np
import time
import math
import rangeSensorFunctions as rangeSensor

# global variables for the youBot
# B,C,D in meters
B = 0.1
C = 0.471
D = 0.30046

FORWARD_VEL =  -2.0
SIDEWARD_VEL =  2.0
ROTATE_VEL =  2.0

STEP = 2.0	# constant for the leaving condition

def getWheelJoints(clientID):
    # Retrieve wheel joint handles:
    wheelJoints = np.empty(4, dtype=np.int)
    wheelJoints.fill(-1)  # front left, rear left, rear right, front right
    res, wheelJoints[0] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep.simx_opmode_oneshot_wait)
    res, wheelJoints[1] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_oneshot_wait)
    res, wheelJoints[2] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep.simx_opmode_oneshot_wait)
    res, wheelJoints[3] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_oneshot_wait)

    return wheelJoints


def forward(meter, clientID):
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
    while distance <= meter:
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

# drives forwards for meter meters as long as there is no obstacle in front, only 2 laser ranges are used here
# returns true if the bot encounterd an obstacle, and false if the bot drove for meter meters
def forwardUntilObstacleFront(meter, clientID, rangeSensorHandles):
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
    while distance <= meter and stop!=True:
        start = time.time()
        # get range sensor data as list of x,y,z,distance
        rangeData = rangeSensor.getSensorData(clientID, rangeSensorHandles)
        if rangeData[342][3] <= 0.3 and rangeData[343][3] <= 0.3:
            stop=True
            break

        x, y, w = odometry(x, y, 0.0, FORWARD_VEL, 0.0, 0.0, dt)
        distance = math.sqrt(x*x+y*y)
        time.sleep(1.0e-06)         # problems with very small time slices -> little delay (if you have a bad angle calculation on your pc try to change this value)
        end = time.time()
        dt = end-start

    # stop moving
    setWheelVelocity(clientID, 0)

    return(stop)

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
    while w <= degree:
        start = time.time()
        x, y, w = odometry(0.0, 0.0, w, 0.0, 0.0, rotationVel, dt)
        time.sleep(1.0e-06)         # problems with very small time slices -> little delay (if you have a bad angle calculation on your pc try to change this value)
        end = time.time()
        dt = end - start

    # stop moving
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], 0, vrep.simx_opmode_oneshot)
    return

def rotateUntilOrientation(clientID, targetOrient):
    rotationVel = ROTATE_VEL
    wheelJoints = getWheelJoints(clientID)
    currentOrient = getOrientation(clientID)
    print("Current Orientation: {} , Target Orientation: {}" .format(currentOrient, targetOrient))
    if currentOrient>0:
        if targetOrient>currentOrient:
            rotationVel *= -1
            startRotating(clientID, rotationVel)
            while targetOrient > currentOrient:
                time.sleep(0.05)
                currentOrient = getOrientation(clientID)

        elif targetOrient<currentOrient:
            rotationVel *= 1
            startRotating(clientID, rotationVel)
            while targetOrient < currentOrient:
                time.sleep(0.05)
                currentOrient = getOrientation(clientID)

    elif currentOrient<0:
        if targetOrient < currentOrient:
            rotationVel *= 1
            startRotating(clientID, rotationVel)
            while targetOrient < currentOrient:
                time.sleep(0.05)
                currentOrient = getOrientation(clientID)

        elif targetOrient > currentOrient:
            rotationVel *= -1
            startRotating(clientID, rotationVel)
            while targetOrient > currentOrient:
                time.sleep(0.05)
                currentOrient = getOrientation(clientID)

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

# returns false if the bot succesfully reached the target and true if the bot encountered an obstacle
def headTowardsModel(clientID, modelName, rangeSensorHandles):
    print("headTowardsModel begin")
    res, objHandle = vrep.simxGetObjectHandle(clientID, modelName, vrep.simx_opmode_oneshot_wait)

    targetPosition = vrep.simxGetObjectPosition(clientID, objHandle, -1, vrep.simx_opmode_oneshot_wait)
    xTarget = targetPosition[1][0]
    yTarget = targetPosition[1][1]
    print ("{}: x= {}, y= {}" .format(modelName,xTarget,yTarget))
    pos, ori = getPos(clientID)

    targetOrientation = calcTargetOrient(clientID, pos[0], pos[1], xTarget, yTarget)
    print("Orientation of target: {}".format(targetOrientation))

    rotateUntilOrientation(clientID, targetOrientation)
    
    dist = calcDistanceToTarget(pos[0], pos[1], xTarget, yTarget)
    case, hit  = forwardUntilObstacleAnywhere(dist, clientID, rangeSensorHandles)
    print("headTowardsModel end with {}".format(case))
    return case, hit

# calculates distance between 2 x,y coordinates
def calcDistanceToTarget(xStart, yStart, xEnd, yEnd):
    distanceToTarget = math.sqrt((xEnd-xStart)*(xEnd-xStart)+(yEnd-yStart)*(yEnd-yStart))
    #print(distanceToTarget)
    return distanceToTarget

def calcTargetOrient(clientID, xStart, yStart, xEnd, yEnd):
    GK = abs(float(yEnd-yStart))
    AK = abs(float(xEnd-xStart))
    #print("Angle: {}".format(math.tan(GK / AK) * 180.0 / math.pi))
    #print("xEnd: {}, xStart: {}, yEnd: {}, yStart: {}".format(xEnd, xStart, yEnd, yStart))

    angle= abs(math.atan2(GK, AK) * 180.0 / math.pi)

    # 4 cases where the target is
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

def wallOrient(clientID, rangeSensorHandles, rayHit):
    rangeData = rangeSensor.getSensorData(clientID, rangeSensorHandles)

    x1 = rangeData[rayHit][0]
    y1 = rangeData[rayHit][1]
    x2 = rangeData[rayHit+10][0]
    y2 = rangeData[rayHit+10][1]
    if(y2 > y1):
        a = calcTargetOrient(clientID, x2,y2,x1,y1)
    else:
        a = calcTargetOrient(clientID, x1,y1,x2,y2)

    rotateUntilOrientation(clientID, a)
    
def detectCorner(clientID, rangeSensorHandles, rayHit, isRight):
    rangeData = rangeSensor.getSensorData(clientID, rangeSensorHandles)

    x1 = rangeData[rayHit][0]
    y1 = rangeData[rayHit][1]
    if(isRight):
        x2 = rangeData[rayHit+15][0]
        y2 = rangeData[rayHit+15][1]

    else:
        x2 = rangeData[rayHit-15][0]
        y2 = rangeData[rayHit-15][1]

    print("x1:",x1,"x2:",x2,"y1:",y1,"y2:",y2)

    if(abs(x1) < abs(x2) - 1):
        return True
    if(abs(y1) < abs(y2) - 1):
        return True

    return False

def forwardUntilCorner(clientID, rangeSensorHandles, isRight):
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
    corner = False
    while not corner:
        start = time.time()
        # get range sensor data as list of x,y,z,distance
        rangeData = rangeSensor.getSensorData(clientID, rangeSensorHandles)

        x, y, w = odometry(x, y, 0.0, FORWARD_VEL, 0.0, 0.0, dt)
        distance = math.sqrt(x*x+y*y)
        time.sleep(1.0e-06)         # problems with very small time slices -> little delay (if you have a bad angle calculation on your pc try to change this value)
        end = time.time()
        dt = end-start
        if(isRight):
            corner = detectCorner(clientID,rangeSensorHandles,665,isRight)
            print("1")
        else:
            print("r")
            corner = detectCorner(clientID, rangeSensorHandles,15,isRight)

    # stop moving
    setWheelVelocity(clientID, 0)

    return(stop)
    
    
# This function calculates the leaving condition for the DistBug algorithm.
# For the calculation the minimum distance to the target needs to be tracked (it will be updated in this function).
# distNextObstacle - The distance to another obstacle arount the robot (not the obstacle the robot is following) (max value if no obstacle around)
# returns True if the leaving condtion holds, falls otherwise and the new minDist
def calcLeavingConditin(minDist, distNextObstacle, clientID):
    leave = False
	
	# calc current dist. to target
    res, objHandle = vrep.simxGetObjectHandle(clientID, "Goal", vrep.simx_opmode_oneshot_wait)
    targetPosition = vrep.simxGetObjectPosition(clientID, objHandle, -1, vrep.simx_opmode_oneshot_wait)
    xTarget = targetPosition[1][0]
    yTarget = targetPosition[1][1]

    pos, ori = getPos(clientID)
    
    dist = calcDistanceToTarget(pos[0], pos[1], xTarget, yTarget)
    
    
    # calc leaving condition
    if dist - distNextObstacle <= minDist - STEP:
        leave = True
    
    
    # set minDist if neccesary
    if dist < minDist:
        mindist = dist
		
		
    return leave, minDist


# This function gets the angle from the robot to the goal and returns the 
# index of the ray pointing to the goal
# Some small tests show that this function has an anomaly by +-4°
def getRayBasedOnAngle(angle, clientID):
    # calculate angle to target
    targetOrientation = calcOrientationToTarget(clientID, "Goal")
    robotOrientation = getOrientation(clientID)

    angleToTarget = substractOrientation(targetOrientation, robotOrientation)


    # choose between left and right sensor and
    # calculate ray pointing to target
    if angleToTarget > 0:   # left sensor
        ray = math.fabs(angleToTarget) / 0.351      # calculate ray as float
        ray = round(ray)                            # round to int
        ray += 342                                  # add first idx of left sensor
    else:                   # right sensor
        ray = (120 - math.fabs(angleToTarget)) / 0.351
        ray = round(ray)

    return ray


def calcOrientationToTarget(clientID, targetName):
    res, objHandle = vrep.simxGetObjectHandle(clientID, targetName, vrep.simx_opmode_oneshot_wait)

    targetPosition = vrep.simxGetObjectPosition(clientID, objHandle, -1, vrep.simx_opmode_oneshot_wait)
    xTarget = targetPosition[1][0]
    yTarget = targetPosition[1][1]
    pos, ori = getPos(clientID)

    targetOrientation = calcTargetOrient(clientID, pos[0], pos[1], xTarget, yTarget)

    return targetOrientation



# This function substracts two orientation. It returns the substracted orientation in a range (-180,+180].
def substractOrientation(orientationA, orientationB):
    newOrientation = orientationA - orientationB

    if newOrientation <= -180:               # new orientation changed from negative to positive
        newOrientation = 180 + (newOrientation + 180)
    elif newOrientation > 180:              # new orientation changed from positive to negative
        newOrientation = -180 + (newOrientation - 180)

    return newOrientation
    
	
	
