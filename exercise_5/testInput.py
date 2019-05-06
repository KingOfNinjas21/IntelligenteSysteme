# import the necessary packages
import numpy as np
import argparse
import cv2

# construct the argument parse and parse the arguments

# load the image
image = cv2.imread("test.png")
image2 = cv2.imread("test.png")

# define the list of boundaries
boundaries = [([0, 230, 0], [20, 255, 20])]

# loop over the boundaries
for (lower, upper) in boundaries:
    # create NumPy arrays from the boundaries
    lower = np.array(lower, dtype="uint8")
    upper = np.array(upper, dtype="uint8")

    # find the colors within the specified boundaries and apply
    # the mask
    mask = cv2.inRange(image, lower, upper)
    output = cv2.bitwise_and(image, image, mask=mask)

    imgray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(imgray, 127, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    cv2.drawContours(image2, contours, 0, (255, 0, 0), 1)
    cv2.imshow("conturs", np.hstack([image2, output]))

    



    '''
    #show contours
    cnts, scnd = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image2, cnts, 4, (0, 255, 0), 1)
    cv2.imshow("blaba", np.hstack([image2, output]))
    cv2.waitKey(0)
    '''

    # calculate center
    M = cv2.moments(contours[0])
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])

    print(cx, cy)

    cv2.waitKey(0)

  
