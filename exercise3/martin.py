# Introduction to Intelligent and Autonomus Systems, UIBK, 2017
#contact:senka.krivic@uibk.ac.at
# 

import vrep
import numpy as np
import time
import math

#def wheelVel(forwBackVel, leftRightVel, rotVel):
#    write your function here

def vel(vx,vy,w0,r,l1,l2):
        vx *= 1000.0
        vy *= 1000.0
        vel1 = 1/r * (1*vx + 1 * vy -(l1+l2)*w0)  
        vel2 = 1/r * (1*vx - 1 * vy +(l1+l2)*w0) 
        vel3 = 1/r * (1*vx - 1 * vy -(l1+l2)*w0) 
        vel4 = 1/r * (1*vx + 1 * vy +(l1+l2)*w0)

        return np.array([vel1,vel3,vel4,vel2])

def drive(wheelVelocities, t, clientID,wheelJoints):
        #print("VEL:",wheelVelocities,"\n")
        vrep.simxPauseCommunication(clientID, True)
        for i in range(0, 4):
            vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],wheelVelocities[i],vrep.simx_opmode_oneshot)

        vrep.simxPauseCommunication(clientID, False)
        time.sleep(t)

        for i in range(0, 4):
            vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],0,vrep.simx_opmode_oneshot)

def aimGoal(robot, goal, ori):
        ak = goal[0] - robot[0]
        gk = goal[1] - robot[1]      
        a = math.atan2(gk,ak) - ori[2] #+ math.pi
        print("ak=",ak,"\ngk=",gk,"\na=",a)
        
        return [0,0,a]
                
                
        
        

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
        res,wheelJoints[2]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rr',vrep.simx_opmode_oneshot_wait)
        res,wheelJoints[3]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fr',vrep.simx_opmode_oneshot_wait)
    
        # set wheel velocity to 0
        for i in range(0, 4):
            vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],0,vrep.simx_opmode_oneshot)
        
        #ste first wheel velocities to 5 m/s for 2 s

        
        r = 50.0 #radius of wheels	
        l1 = 300.46/2.0 #D/2
        l2 = 471.0/2.0 #C/2
        """
        vx = 0  
        vy = 0
        w0 = 1
	        
        vel1 = 1/r * (1*vx + 1 * vy -(l1+l2)*w0)  
        vel2 = 1/r * (1*vx  -1 * vy +(l1+l2)*w0) 
        vel3 = 1/r * (1*vx  -1 * vy -(l1+l2)*w0) 
        vel4 = 1/r * (1*vx + 1 * vy +(l1+l2)*w0) 
        """

        res, base=vrep.simxGetObjectHandle(clientID,'youBot_center',vrep.simx_opmode_oneshot_wait)
        res, base_pos=vrep.simxGetObjectPosition(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
        res, base_orient= vrep.simxGetObjectOrientation(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
        vrep.simxGetPingTime(clientID)

        #print(base,"\n",base_pos,"\n",base_orient,"\n")

        res = vrep.simxSetIntegerSignal(clientID, 'handle_xy_sensor', 2, vrep.simx_opmode_oneshot);

        vrep.simxSetIntegerSignal(clientID, 'displaylasers', 1, vrep.simx_opmode_oneshot);

        res, hokuyo1 = vrep.simxGetObjectHandle(clientID, 'fastHokuyo_sensor1', vrep.simx_opmode_oneshot_wait)
        res, hokuyo2 = vrep.simxGetObjectHandle(clientID, 'fastHokuyo_sensor2', vrep.simx_opmode_oneshot_wait)



        res, goalBase = vrep.simxGetObjectHandle(clientID,'Goal', vrep.simx_opmode_oneshot_wait)
        res, goal = vrep.simxGetObjectPosition(clientID, goalBase, -1, vrep.simx_opmode_oneshot_wait)  
        #print(goal)

        #turn around in staart pos
        #wheelVelocities = vel(0,0,1.0,r,l1,l2)
        #drive(wheelVelocities, math.pi, clientID,wheelJoints)

        #aim for goal 
        angle = aimGoal(base_pos, goal, base_orient)
        wheelVelocities = vel(0,0,1.0,r,l1,l2)
        drive(wheelVelocities, angle[2], clientID, wheelJoints)  
        
        res, aux, auxD = vrep.simxReadVisionSensor(clientID, hokuyo1, vrep.simx_opmode_streaming)
        res, aux, auxD = vrep.simxReadVisionSensor(clientID, hokuyo1, vrep.simx_opmode_buffer)
        print(auxD)
        #print(math.atan2(-3,3))       
        #print(angle[2])
        """
        #7 straight 
        
        wheelVelocities = vel(-0.5,0,0,r,l1,l2)
        drive(wheelVelocities, 14.0,clientID, wheelJoints)
        #turn 
        wheelVelocities = vel(0,0,1.0,r,l1,l2)
        drive(wheelVelocities, math.pi, clientID,wheelJoints)

        #7 straight 
        
        wheelVelocities = vel(-0.5,0,0,r,l1,l2)
        drive(wheelVelocities, 14, clientID,wheelJoints)
        #turn 
        wheelVelocities = vel(0,0,1.0,r,l1,l2)
        drive(wheelVelocities, math.pi,clientID,wheelJoints)

        #7 straight 
        
        wheelVelocities = vel(-0.5,0,0,r,l1,l2)
        drive(wheelVelocities, 14, clientID,wheelJoints)
        #turn 
        wheelVelocities = vel(0,0,1.0,r,l1,l2)
        drive(wheelVelocities, math.pi, clientID,wheelJoints)

        #7 straight 
        
        wheelVelocities = vel(-0.5,0,0,r,l1,l2)
        drive(wheelVelocities, 14, clientID,wheelJoints)
        #turn 
        wheelVelocities = vel(0,0,1.0,r,l1,l2)
        drive(wheelVelocities, math.pi, clientID,wheelJoints)

        """
        res,base=vrep.simxGetObjectHandle(clientID,'youBot_center',vrep.simx_opmode_oneshot_wait)
        base_pos=vrep.simxGetObjectPosition(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
        base_orient= vrep.simxGetObjectOrientation(clientID, base, -1, vrep.simx_opmode_oneshot_wait)
        vrep.simxGetPingTime(clientID)

       # print(base,"\n",base_pos,"\n",base_orient,"\n")

       
        

        
        # set wheel velocity to 0
        for i in range(0, 4):
            vrep.simxSetJointTargetVelocity(clientID,wheelJoints[i],0,vrep.simx_opmode_oneshot)
        
        # Stop simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)
    
        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')

if __name__ == "__main__": main()
