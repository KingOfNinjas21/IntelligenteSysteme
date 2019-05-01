import array
import numpy as np
import cv2
import vrep
import movementFunctions as move

from PIL import Image


# constans for color boundaries
boundariesGreen = ([50, 200, 50], [75, 255, 75])



'''
converts an image from vrep sensor to cv2 format.
sesorImg: the image from the sensor
res: the resolution as array
'''
def convertToCv2Format (sensorImg, res):
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
def calcCenter (contours):
    M = cv2.moments(contours[0])
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])

    return cx, cy

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
    ret, thresh = cv2.threshold(imgray, 127, 255, 0)

    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    return contours

def exercise4_action(clientID, youBotCam):
    counter = 1

    while counter <= 5:
        # get further images from vision sensor
        err, res, image = vrep.simxGetVisionSensorImage(clientID, youBotCam, 0, vrep.simx_opmode_buffer)

        if err == vrep.simx_return_ok:
            # do some image stuff ----------------------------------------------------------------------------------

            cv2Image = convertToCv2Format(image, res)
            cv2ImageCopy = cv2Image

            imageContours = getContours(cv2Image, boundariesGreen)
            cv2.drawContours(cv2ImageCopy, imageContours, 0, (255, 0, 0), 1)
            cv2.imshow("Current Image of youBot", cv2ImageCopy)

            # calculate center of green blob
            x, y = calcCenter(imageContours)

            print("Current center of the green blob: x={} y={}".format(x, y))

            cv2.waitKey(0)  # key to get out of waiting is ESC

            # end some image stuff ---------------------------------------------------------------------------------

        move.forward(0.1, clientID)

        counter += 1

