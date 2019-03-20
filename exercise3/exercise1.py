# Introduction to Intelligent and Autonomus Systems, UIBK, 2017
#contact:senka.krivic@uibk.ac.at
# 

import vrep
import numpy as np
import time
import math


B = 0.1
C = 0.471
D = 0.30046

def main():
    print ('Program started')
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19997,True,True, 2000,5)

    if clientID!=-1:
        print ('Connected to remote API server')
    
        emptyBuff = bytearray()
    
        # Start the simulation:
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait)
    
        # initiaize robot
        # Retrieve wheel joint handles:
        wheelJoints=np.empty(4, dtype=np.int); wheelJoints.fill(-1) #front left, rear left, rear right, front right
        res,wheelJoints[0]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fl',vrep.simx_opmode_oneshot_wait)
        res,wheelJoints[1]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rl',vrep.simx_opmode_oneshot_wait)
        res,wheelJoints[2]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fr',vrep.simx_opmode_oneshot_wait)
        res,wheelJoints[3]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rr',vrep.simx_opmode_oneshot_wait)

        printPos(clientID)


        for i in range(0, 4):
            vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],0,vrep.simx_opmode_oneshot)

        vrep.simxPauseCommunication(clientID, True)
        for i in range(0, 4):
            vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],wheelVel(0.0, 2.0, 0.0)[i],vrep.simx_opmode_oneshot)
        vrep.simxPauseCommunication(clientID, False)


        distX = 0
        dt = 0.0
        while(distX < 2.0):
            start = time.time()
            x, y, w = odometry(0.0, 0.0, 0.0, 0.0, 2.0, 0.0, dt)
            distX += x
            #printPos(clientID)
            end = time.time()
            dt = end - start


        for i in range(0, 4):
            vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],0,vrep.simx_opmode_oneshot)

        printPos(clientID)


        # Stop simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)
    
        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')

#def moveForward(meter, wheelJoints, clientID):
#    move(wheelVel(16, 0, 0), meter*1.55, wheelJoints, clientID)
	

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

def printPos(clientID):
    res, base = vrep.simxGetObjectHandle(clientID, 'youBot_center', vrep.simx_opmode_oneshot_wait)
    base_pos = vrep.simxGetObjectPosition(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
    base_orient = vrep.simxGetObjectOrientation(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
    vrep.simxGetPingTime(clientID)  # make sure that all streaming data has reached the client at least once

    print "Position: ", base_pos[1]
    print "Orientation: ", base_orient[1]

#def move(wheelVelocities, sleapTime, wheelJoints, clientID):
	    # set wheel velocity to 0
#    for i in range(0, 4):
#        vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],0,vrep.simx_opmode_oneshot)
            
		# set wheel velocity to given value
#    vrep.simxPauseCommunication(clientID, True)
#    for i in range(0, 4):
#        vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],wheelVelocities[i],vrep.simx_opmode_oneshot)
#    vrep.simxPauseCommunication(clientID, False)
#    time.sleep(sleapTime)
        
        # set wheel velocity to 0
#    for i in range(0, 4):
#        vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],0,vrep.simx_opmode_oneshot)


if __name__ == "__main__": main()
