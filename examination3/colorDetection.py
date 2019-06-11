import array
import numpy as np
import cv2
import vrep
import movementFunctions as move
import math
import aStar

from PIL import Image

# constans for color boundaries
boundariesGreen = ([0, 200, 0], [75, 255, 75])
boundariesYellow = ([0, 190, 190], [80, 255, 255])
boundariesBlue = ([190, 0, 0], [255, 85, 85])
boundariesRed = ([0, 0, 190], [86, 86, 255])
boundiresOrange = ([0,100,231],[35,150,255])

#right now not needed boundaries list: colors = [boundariesRed, boundariesYellow, boundariesBlue, boundariesGreen, boundiresOrange]
colors = [boundariesRed]

def test(image):
    imageCopy = image
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]



    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)[0]

    cv2.drawContours(imageCopy, cnts, 0, (255, 255, 255), 1)
    cv2.imshow("Current Image of youBot", imageCopy)
    cv2.waitKey(0)



def egocentricToGlobal(ego, clientID):
    x = ego[0]
    y = ego[1]
    pos, orient = move.getPos(clientID)

    alpha = move.getOrientation(clientID)/180.0*math.pi+math.pi/2.0

    rotationMatrix = [[math.cos(alpha), -math.sin(alpha)],
                      [math.sin(alpha), math.cos(alpha)]]

    
    newVec = np.dot(np.array(rotationMatrix), np.array([x, y]))

    x = newVec[0]
    y = newVec[1]

    xBot = pos[0]
    yBot = pos[1]



    return [x+xBot, y+yBot]



def globalToEgocentric(globCorrd, clientID):
    pos, orient = move.getPos(clientID)
    egoVec = [globCorrd[0]-pos[0], globCorrd[1]-pos[1]]

    alpha = move.getOrientation(clientID)/180.0*math.pi+math.pi/2.0

    rotationMatrix = [[math.cos(-alpha), -math.sin(-alpha)],
                      [math.sin(-alpha), math.cos(-alpha)]]
    newVec = np.dot(np.array(rotationMatrix), np.array(egoVec))

    return newVec

'''
converts an image from vrep sensor to cv2 format.
sesorImg: the image from the sensor
res: the resolution as array
'''


def convertToCv2Format(sensorImg, res):
    # transformation to byte array
    image_byte_array = array.array('b', sensorImg)

    # transformation to opencv2 image
    image_buffer = Image.frombuffer("RGB", (res[0], res[1]), np.asarray(image_byte_array), "raw", "RGB", 0, 1)
    img = np.asarray(image_buffer)

    # Convert RGB to BGR and flip
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.flip(img, 0)

    return img


'''
calculates the center for given contours of an object in an image
'''


def calcCenter(contour):
    M = cv2.moments(contour)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])

    return [cx, cy, 1]


"""
gives us the bottom most point of an Object
"""


def getBottom(cnt):
    cnt = cnt
    bottom = (cnt[cnt[:, :, 1].argmax()][0])
    bottom = np.append(bottom, 1)

    #extLeft =

    return bottom


def getBlobsGlobal(img, homoMatrix, clientID):
    imgCopy = img
    iC = img
    points = []

    for c in colors:
        img = imgCopy
        cnts = getContours(img, c)
        #print(cnts)
        
        for k in cnts:
            newPoint = np.dot(homoMatrix, getBottom(k))
            newPoint = newPoint / newPoint[2]
            newPoint = egocentricToGlobal(newPoint, clientID)
             #print(newPoint)

            points.append((newPoint, c))
        cv2.drawContours(iC, cnts, -1, (255, 255, 255), 1)
        #cv2.imshow("Current Image of youBot", iC)
        #cv2.waitKey(0)
           
    
    return points

def findAllBlobs(clientId, youBotCam, homoMatrix):

    currentDegree = 0
    blobList = []
    #we will rotate for 180 degree for spotting the blobs
    i = 0
    while(i<4):
        i+=1

        err, res, image = vrep.simxGetVisionSensorImage(clientId, youBotCam, 0, vrep.simx_opmode_buffer)
        

        if err == vrep.simx_return_ok:
            # do some image stuff ----------------------------------------------------------------------------------

            cv2Image = convertToCv2Format(image, res)

            tempBlobs = getBlobsGlobal(cv2Image, homoMatrix, clientId)

            count = 0
            for tb in tempBlobs:
                count = 0
                for b in blobList:
                    if move.isSamePoint(tb[0], b[0]) and tb[1] == b[1]:
                        count = count + 1
                
                if count == 0:
                    print("added", tb, " to blobList")
                    blobList.append(tb)

        move.rotate(10, clientId, True)

    
    return blobList



'''
Gets a raw cv2 image, process it and returns the contours of the given color boundaries 
'''


def getContours(image, boundaries):
    # create NumPy arrays from the boundaries
    lower = np.array(boundaries[0], dtype="uint8")
    upper = np.array(boundaries[1], dtype="uint8")

    # find the colors within the specified boundaries and apply the mask

    mask = cv2.inRange(image, lower, upper)
    output = cv2.bitwise_and(image, image, mask=mask)

    # process image
    imgray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(imgray, 0, 255, cv2.THRESH_BINARY)  # 127, 255, 0)

    #cv2.imshow("THRresh", mask)
    #cv2.waitKey(0)

    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    return contours


'''
This is the action to be called for the exercise 4 of the proseminar
1. Retrieves image 
2. processes it and calculates the center of the green blob
3. prints the center of the green blob and waits for the ESC-key 
4. moves 0.1 meters and begins again at 1. until this loop happened 5 times
'''


def exercise4_action(clientID, youBotCam):
    counter = 1

    while counter < 5:
        # get further images from vision sensor
        err, res, image = vrep.simxGetVisionSensorImage(clientID, youBotCam, 0, vrep.simx_opmode_buffer)

        if err == vrep.simx_return_ok:
            # do some image stuff ----------------------------------------------------------------------------------

            cv2Image = convertToCv2Format(image, res)
            cv2ImageCopy = cv2Image

            imageContours = getContours(cv2Image, boundariesBlue)

            cv2.drawContours(cv2ImageCopy, imageContours, 0, (255, 0, 0), 1)
            cv2.imshow("Current Image of youBot", cv2ImageCopy)

            # calculate center of green blob
            x, y = calcCenter(imageContours[0])

            print("Current center of the green blob: x={} y={}".format(x, y))

            cv2.waitKey(0)  # key to get out of waiting is ESC

            # end some image stuff ---------------------------------------------------------------------------------

        move.forward(0.1, clientID)

        counter += 1


'''
This method takes a picture of a chessboard and uses the global coordinates of the chessboard corners to calculate the H-Matrix of the camera.
Make sure the robot camera sees the whole chessboard.
chessboard_corners - The global coordinates of the chessboard.
'''
def get_H_matrix(chessboard_corners, youBotCam, clientID):
    print("Begin calculation of H-matrix, please wait ...")
    err, res, image = vrep.simxGetVisionSensorImage(clientID, youBotCam, 0, vrep.simx_opmode_buffer)
    image = convertToCv2Format(image, res)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    found, prime_corners = cv2.findChessboardCorners(image, (3, 4))

    prime_corners = addOne(prime_corners)

    # convert all global corners of the chessboard in egocentric world space
    ego_corners = []
    for gc in chessboard_corners:
        newCorner = globalToEgocentric(gc, clientID)
        ego_corners.append(newCorner)

    # add a 1 in every row (globalToEgocentric only returns x,y coordinates
    ego_corners = addOne(ego_corners)

    # convert ego_corners in numpy array
    ego_corners = np.asarray(ego_corners)

    # calculate H-matrix
    A = getA(prime_corners, ego_corners)
    return getH(A)


# adds a one at the end of every row
def addOne(matrix):
    new = []
    for i in range(len(matrix)):
        row = np.append(matrix[i], 1)
        new.append(row)
    return np.asarray(new)


# retrieve H-matrix via the A-matrix
def getH(A):
    # the matrix we want is the last row of vh which needs then to be reshaped
    u, s, vh = np.linalg.svd(A, full_matrices=False)
    h = vh[8]
    H = np.reshape(h, (3, 3))
    return H


# retrieve A-matrix via prime corners and global corners
# prime_corners = points in screen (picture) space
# global_corners = points in world space
def getA(prime_corners, global_corners):
    A = []
    for i in range(len(prime_corners)):
        p1 = [-prime_corners[i][0], -prime_corners[i][1], -1, 0, 0, 0, prime_corners[i][0] * global_corners[i][0],
              prime_corners[i][1] * global_corners[i][0], global_corners[i][0]]
        p2 = [0, 0, 0, -prime_corners[i][0], -prime_corners[i][1], -1, global_corners[i][1] * prime_corners[i][0],
              global_corners[i][1] * prime_corners[i][1], global_corners[i][1]]

        A.append(np.asarray(p1))
        A.append(np.asarray(p2))

    return np.asarray(A)


# calculates a path from the youBotPos to the goalPos and drives to it
def driveThroughPath(obstacleCoordinates, youBotPos, goalPos, clientID):
    # retrieve an AStar_Solver object
    a = aStar.AStar_Solver(youBotPos, goalPos, obstacleCoordinates)
    print("Starting AStar algorithm...")
    a.Solve()
    print("Found path: ")
    for p in a.path:
        print(p)

    print("\n")

    # go through path list and move from point to point
    for p in range(1, len(a.path)):
        print("next target: ", a.path[p])
        move.moveToCoordinate(a.path[p][0], a.path[p][1], clientID)

# sort the blobs from right to left (selectionSort)
def sortBlobsByRad(blobs):
    blobsCopy = blobs[:]

    blobsSorted = [([],([],[]))]

    while len(blobsCopy) > 0:
        min = blobsCopy[0]
        for blob in blobsCopy:
            if getBlobXAngle(blob) < getBlobXAngle(min):
                min = blob

        blobsSorted.append(min)
        blobsCopy.remove(min)

    return blobsSorted[1:]


def getBlobXAngle(blob):
    return math.atan(blob[0][1] / blob[0][0])
