# import the necessary packages
import numpy as np
import argparse
import cv2

# construct the argument parse and parse the arguments

# load the image
image = cv2.imread("color_detection_blue_version.jpg")
image2 = cv2.imread("color_detection_blue_version.jpg")

# define the list of boundaries
boundaries = [([17, 15, 100], [50, 56, 200])]

# loop over the boundaries
for (lower, upper) in boundaries:
    # create NumPy arrays from the boundaries
    lower = np.array(lower, dtype="uint8")
    upper = np.array(upper, dtype="uint8")

    # find the colors within the specified boundaries and apply
    # the mask
    mask = cv2.inRange(image, lower, upper)
    output = cv2.bitwise_and(image, image, mask=mask)

    imgray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(imgray, 127, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    cv2.drawContours(image, contours, -1, (0, 255, 0), 3)

    # show the images
    #cv2.imshow("images", np.hstack([image, output]))
    cnts, scnd = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image2, cnts, 4, (0, 255, 0), 1)
    cv2.imshow("blaba", np.hstack([image2, output]))
    cv2.waitKey(0)