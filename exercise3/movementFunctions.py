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
                break

        x, y, w = odometry(x, y, 0.0, FORWARD_VEL, 0.0, 0.0, dt)
        distance = math.sqrt(x*x+y*y)
        time.sleep(1.0e-06)         # problems with very small time slices -> little delay (if you have a bad angle calculation on your pc try to change this value)
        end = time.time()
        dt = end-start

    # stop moving
    setWheelVelocity(clientID, 0)

    return(stop)

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
    case = forwardUntilObstacleAnywhere(dist, clientID, rangeSensorHandles)
    print("headTowardsModel end with {}".format(case))
    return case

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