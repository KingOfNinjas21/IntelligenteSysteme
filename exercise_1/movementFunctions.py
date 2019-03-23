import vrep
import numpy as np
import time
import math

# global variables for the youBot
# B,C,D in meters
B = 0.1
C = 0.471
D = 0.30046

FORWARD_VEL = 2.0
SIDEWARD_VEL = 2.0
ROTATE_VEL = 0.25

def getWheelJoints(clientID):
    # Retrieve wheel joint handles:
    wheelJoints = np.empty(4, dtype=np.int)
    wheelJoints.fill(-1)  # front left, rear left, rear right, front right
    res, wheelJoints[0] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep.simx_opmode_oneshot_wait)
    res, wheelJoints[1] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_oneshot_wait)
    res, wheelJoints[2] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep.simx_opmode_oneshot_wait)
    res, wheelJoints[3] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_oneshot_wait)

    return wheelJoints


def customMovement(time_arg, clientID, forw_back_vel, left_right_vel, rotvel):
    wheelVelocities = wheelVel(forw_back_vel, left_right_vel, rotvel)
    startMovement(time_arg, clientID, wheelVelocities)

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
    while distance < meter:
        start = time.time()
        x, y, w = odometry(0.0, 0.0, 0.0, FORWARD_VEL, 0.0, 0.0, dt)
        distance += math.sqrt(x*x+y*y)
        end = time.time()
        dt = end -start

    # stop moving
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], 0, vrep.simx_opmode_oneshot)



def rotate(degree, clientID):
    # set velocety to 0
    wheelJoints = getWheelJoints(clientID)
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], 0, vrep.simx_opmode_oneshot)

    # start moving
    vrep.simxPauseCommunication(clientID, True)
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], wheelVel(0.0, 0.0, ROTATE_VEL)[i],
                                        vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID, False)

    # continuously check traveled distance
    turnedDigree = 0.0
    dt = 0.0
    printCounter = 0
    while turnedDigree < degree/100:
        start = time.time()
        x, y, w = odometry(0.0, 0.0, 0.0, 0.0, 0.0, ROTATE_VEL, dt)
        turnedDigree += w

        if (100*turnedDigree)%100 >= printCounter:
            printCounter += 1
            print(turnedDigree)

        end = time.time()
        dt = end -start

    print(turnedDigree)

    # stop moving
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], 0, vrep.simx_opmode_oneshot)
    return


def oldforward(speed, distance, clientID):
    print ("Begin forward at ")
    printPos(clientID)
    wheelJoints = getWheelJoints(clientID)
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], 0, vrep.simx_opmode_oneshot)

    vrep.simxPauseCommunication(clientID, True)
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], wheelVel(-speed, 0.0, 0.0)[i],
                                        vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID, False)

    distanceTravelled = 0
    dt = 0.0
    while distanceTravelled < distance:
        start = time.time()
        x, y, w = odometry(0.0, 0.0, 0.0, 4.0, 0.0, 0.0, dt)
        distanceTravelled+=math.sqrt(x*x+y*y)
        #printPos(clientID)
        end = time.time()
        dt = end - start

    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], 0, vrep.simx_opmode_oneshot)
    print ("End forward at ")
    printPos(clientID)


def rotateDegrees(degrees, clientID, rotateSpeed):
    rad = degrees * math.pi / 180.0
    tempTime = rad / rotateSpeed
    print(tempTime)
    wheelVelocities=calcVelocitiesForRotation(rotateSpeed)
    startMovement(tempTime, clientID, wheelVelocities)


def calcVelocitiesForRotation(rotateSpeed):
    #what C and D is, have a look at YouBotDetailedSpecifiations

    R = 50.0 #radius of one wheel
    L1 = 150.23 # D/2
    L2 = 235.5 # C/2
    vx = 0 #speedForXandY
    vy = 0 #speedForXandY

    vel = [0.0]*4
    vel[0] = 1 / R * (1 * vx + 1 * vy - (L1 + L2) * rotateSpeed) # front left
    vel[1] = 1 / R * (1 * vx - 1 * vy + (L1 + L2) * rotateSpeed) # front right
    vel[2] = 1 / R * (1 * vx - 1 * vy - (L1 + L2) * rotateSpeed) # rear left
    vel[3] = 1 / R * (1 * vx + 1 * vy + (L1 + L2) * rotateSpeed) # rear right
    print (vel)
    return vel


def startMovement(time_arg, clientID, wheelVelocities):
    # initialize robot
    wheelJoints = getWheelJoints(clientID)

    # set wheel velocity to 0
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], 0, vrep.simx_opmode_oneshot)

    vrep.simxPauseCommunication(clientID, True)
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], wheelVelocities[i], vrep.simx_opmode_oneshot)
    vrep.simxPauseCommunication(clientID, False)
    time.sleep(time_arg)

    # set wheel velocity to 0
    for i in range(0, 4):
        vrep.simxSetJointTargetVelocity(clientID, wheelJoints[i], 0, vrep.simx_opmode_oneshot)


def printPos(clientID):
    res, base = vrep.simxGetObjectHandle(clientID, 'youBot_center', vrep.simx_opmode_oneshot_wait)
    base_pos = vrep.simxGetObjectPosition(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
    base_orient = vrep.simxGetObjectOrientation(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
    vrep.simxGetPingTime(clientID)  # make sure that all streaming data has reached the client at least once

    print("Position: ", base_pos[1])
    print("Orientation: ", base_orient[1])

def getPos(clientID):
    res, base = vrep.simxGetObjectHandle(clientID, 'youBot_center', vrep.simx_opmode_oneshot_wait)
    base_pos = vrep.simxGetObjectPosition(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
    base_orient = vrep.simxGetObjectOrientation(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
    return base_pos[1], base_orient[1]

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
    vf = forwBackVel * B/2.0
    vr = leftRightVel * B/2.0
    w0 = 2.0/(C+D) * rotVel * B/2.0

    return vf, vr, w0
