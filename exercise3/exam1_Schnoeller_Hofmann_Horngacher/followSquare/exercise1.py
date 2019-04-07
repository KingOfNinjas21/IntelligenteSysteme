# Introduction to Intelligent and Autonomus Systems, UIBK, 2017
#contact:senka.krivic@uibk.ac.at
# 

import vrep
import numpy as np
import time
import math
import movementFunctions as move


def main():
    print ('Program started')
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 2000, 5)

    if clientID!=-1:
        print ('Connected to remote API server')

        # Start the simulation:
        vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot_wait)


        # start demo ---------------------------------------------------------------------------------------------------

        print("startPos:")
        move.printPos(clientID)

        move.forward(5, clientID)
        move.rotate(90, clientID)
        move.printPos(clientID)

        move.forward(5, clientID)
        move.rotate(90, clientID)
        move.printPos(clientID)

        move.forward(5, clientID)
        move.rotate(90, clientID)
        move.printPos(clientID)

        move.forward(5, clientID)
        move.rotate(90, clientID)
        move.printPos(clientID)

        print("endPos:")
        move.printPos(clientID)

        # end demo -----------------------------------------------------------------------------------------------------


        # Stop simulation:
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait)

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')



def calcDistanceToTarget(xStart, yStart, xEnd, yEnd):
    return math.sqrt((xEnd-xStart)*(xEnd-xStart)-(yEnd-yStart)*(yEnd-yStart))


if __name__ == "__main__": main()
