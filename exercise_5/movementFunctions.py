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

def getCameraPos(clientID):
    res, base = vrep.simxGetObjectHandle(clientID, 'rgbdSensor', vrep.simx_opmode_oneshot_wait)
    base_pos = vrep.simxGetObjectPosition(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
    base_orient = vrep.simxGetObjectOrientation(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
    return base_pos[1], base_orient[1]

# returns the youbot orientation in degree
def getOrientation(clientID):
    res, base = vrep.simxGetObjectHandle(clientID, 'youBot_center', vrep.simx_opmode_oneshot_wait)
    base_orient = vrep.simxGetObjectOrientation(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
    return base_orient[1][2]*180.0/math.pi

# returns the rgbd camera orientation in degree
# side note: bot degree| camera degree
#               0      |   145
#               90     |   -125
#             -180     |   -35
#              -90     |   55
def getCameraOrientation(clientID):
    res, base = vrep.simxGetObjectHandle(clientID, 'rgbdSensor', vrep.simx_opmode_oneshot_wait)
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


# This function compares two points with some latitude
def isSamePoint(pointA, pointB):
    latituteRadius = 0.3

    if getDistanceBetweenPoints(pointA, pointB) <= latituteRadius:
        return True

    return False

# This function returns the distance between two points
def getDistanceBetweenPoints(pointA, pointB):
    xA = pointA[0]
    yA = pointA[1]
    xB = pointB[0]
    yB = pointB[1]

    # vector from A to B
    xNew = xB - xA
    yNew = yB - yA

    # length of vector
    dist = math.sqrt(xNew*xNew + yNew*yNew)

    return dist





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


# This function calculates the traveld distance in a given timeslice. Only works if the robot drives forward
# Make sure to track dt accurate
# x, y the start point of this timeslice (init with 0.0, 0.0 if you want to track the distance from a sertain point and update it with the returned values)
def calcTraveldDistance(x, y, dt):
    x, y, w = odometry(x, y, 0.0, FORWARD_VEL, 0.0, 0.0, dt)
    distance = math.sqrt(x * x + y * y)
    time.sleep(1.0e-06)                     # problems with very small time slices -> little delay (if you have a bad angle calculation on your pc try to change this value)

    return x, y, distance


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

    rotateSys(degree, clientID, rotRight)
    return

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
    if isRotRight:
        degree *= -1

    startOrient = getOrientation(clientID)
    goalOrient = substractOrientation(startOrient, -degree)

    # start rotation
    rotateUntilOrientation(clientID, goalOrient)

# rotates the youBot until he reached a certain orientation value
def rotateUntilOrientation(clientID, targetOrient):
    # get needed youBot information's
    rotationVel = ROTATE_VEL
    wheelJoints = getWheelJoints(clientID)
    currentOrient = getOrientation(clientID)
    safty = False
    #print("Orientations for rotation: Current Orientation = {} , Target Orientation = {}" .format(currentOrient, targetOrient))

    # The following are all possibilities that can happen when rotating until a certain orientation

    if(currentOrient > 0 and targetOrient < 0 and abs(targetOrient - currentOrient)> 180):
        rotationVel *= -1
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
    setWheelVelocity(clientID, 0)

    return

# starts rotating with a specific rotation velocity rotationVel
def startRotating(clientID, rotationVel):
    wheelJoints = getWheelJoints(clientID)

    # set velocety to 0
    setWheelVelocity(clientID, 0)

    # start rotating
    vrep.simxPauseCommunication(clientID, True)
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], wheelVel(0.0, 0.0, rotationVel)[i],
                                        vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID, False)

