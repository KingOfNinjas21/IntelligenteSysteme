import array
import numpy as np
import cv2
import vrep
import movementFunctions as move
import math

from PIL import Image

# constans for color boundaries
boundariesGreen = ([0, 200, 0], [75, 255, 75])
boundariesYellow = ([0, 190, 190], [80, 255, 255])
boundariesBlue = ([190, 0, 0], [255, 85, 85])
boundariesRed = ([0, 0, 190], [86, 86, 255])
boundiresOrange = ([0,100,231],[35,150,255])

colors = [boundariesRed, boundariesYellow, boundariesBlue, boundariesGreen, boundiresOrange]


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

    return cx, cy


"""
gives us the bottom most point of an Object
"""


def getBottom(cnt):
    cnt = cnt
    bottom = (cnt[cnt[:, :, 1].argmax()][0])
    bottom = np.append(bottom, 1)

    return bottom


def getBlobsGlobal(img, homoMatrix, clientID):
    imgCopy = img
    iC = img
    points = []
    
    #cv2.imshow("Current Image of youBot", imgCopy)
    #cv2.waitKey(0)


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
    while(currentDegree < 150):
        currentDegree = currentDegree + 30

        err, res, image = vrep.simxGetVisionSensorImage(clientId, youBotCam, 0, vrep.simx_opmode_buffer)
        

        if err == vrep.simx_return_ok:
            # do some image stuff ----------------------------------------------------------------------------------

            cv2Image = convertToCv2Format(image, res)
            cv2ImageCopy = cv2Image

            #cv2.imshow("Current Image of youBot", cv2Image)
            #cv2.waitKey(0)

            tempBlobs = getBlobsGlobal(cv2Image, homoMatrix, clientId)

            count = 0
            for tb in tempBlobs:
                count = 0
                for b in blobList:
                    if move.isSamePoint(tb[0], b[0]) and tb[1] == b[1]:
                        count = count + 1
                
                if count == 0:
                    print("add", tb)
                    blobList.append(tb)

        move.rotate(10,clientId,True)

    
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

    while counter <= 5:
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

