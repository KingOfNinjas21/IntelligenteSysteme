import array
import numpy as np
import cv2
import vrep
import movementFunctions as move
import main

from PIL import Image

# constans for color boundaries
boundariesGreen = ([50, 200, 50], [75, 255, 75])
boundariesYellow = ([50, 210, 210], [80, 255, 255])
boundariesBlue = ([204, 45, 45], [255, 85, 85])
boundariesRed = ([45, 45, 205], [86, 86, 255])

colors = [boundariesRed, boundariesYellow, boundariesBlue, boundariesGreen]

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
    bottom = tuple(cnt[cnt[:, :, 1].argmax()][0])
    print(bottom)
    return bottom


def getBlobsGlobal(img, homoMatrix, clientID):
    imgCopy = img
    points = []
    
    for c in colors:
        img = imgCopy
        cnts = getContours(img, c)

        for k in cnts:
            newPoint = np.dot(homoMatrix, getBottom(k))
            newPoint = newPoint / newPoint[2]
            newPoint = main.egocentricToGlobal(newPoint, clientID)
            points.append((newPoint, c))

    return points

def findAllBlobs(clientId, youBotCam, homoMatrix):

    currentDegree = 0
    blobList = []
    #we will rotate for 180 degree for spotting the blobs
    while(currentDegree < 180):
        currentDegree = currentDegree + 10

        err, res, image = vrep.simxGetVisionSensorImage(clientId, youBotCam, 0, vrep.simx_opmode_buffer)

        if err == vrep.simx_return_ok:
            # do some image stuff ----------------------------------------------------------------------------------

            cv2Image = convertToCv2Format(image, res)
            cv2ImageCopy = cv2Image

            tempBlobs = getBlobsGlobal(cv2Image, homoMatrix, clientId)
            count = 0
            for tb in tempBlobs:
                count = 0
                for b in blobList:
                    if move.isSamePoint(tb[0], b[0]):
                        count = count + 1

                if count == 0:
                    blobList.append(tb)

        print(blobList)
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

    # cv2.imshow("THRresh", mask)
    # cv2.waitKey(0)

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

