import array
import numpy as np
import cv2

from PIL import Image


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

