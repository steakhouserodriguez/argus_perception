#!/usr/bin/python

import cv2
import cv
import time
import numpy
import wx
import math

from numpy import arange, array, ones, linalg
import numpy as np

from plotter import *


#############################################################################
# mouse callback

def onMouseGray(event, x, y, flags, param):
    print(
        "gray position: (" + str(x) + ", " + str(y) + "), value="
        + str(grayFrame[y][x]))

#############################################################################
# mouse callback
def onMouseDiff(event, x, y, flags, param):
    print(
        "diff position: (" + str(x) + ", " + str(y) + "), value="
        + str(difference[y][x]))


threshVal = 0

def onSliderChange(position):
    global threshVal
    print("Slider moved to " + str(position))
    threshVal = position


if __name__ == '__main__':
    print "Starting..."
    global grayFrame, difference, threshVal

    cap = cv2.VideoCapture(1)
    if not cap:
        print "could not capture from camera!"
        exit(1)

        cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, 480)

    retval, frame = cap.read()
 #    thresholdedImage = frame.copy()
    oldframe = frame.copy()
    oldframe = cv2.cvtColor(oldframe, cv2.COLOR_RGB2GRAY)

    cv2.namedWindow("original")
    cv2.namedWindow("blurred")
    cv2.namedWindow("diff")
    cv2.namedWindow("thresh")

    cv.SetMouseCallback("blurred", onMouseGray, None)
    cv.SetMouseCallback("diff", onMouseDiff, None)
    cv2.createTrackbar("threshold", "thresh", 0, 1000, onSliderChange)

    s = SimplePlot()
    while True:
        retval, frame = cap.read()
        #print "type of frame: " + str(type(frame[0][0]))
        grayFrame = frame.copy()
        grayFrame = cv2.cvtColor(grayFrame, cv2.COLOR_RGB2GRAY, grayFrame, 32)
        print "type:" + str(type(grayFrame))
        print "grayFrame dtype:" + str(grayFrame.dtype)

        grayFrame = cv2.GaussianBlur(grayFrame, (9,9), 0)
        grayFrame = grayFrame / 255
        #grayFrame = grayFrame / 128.0
        print "grayFrame dtype:" + str(grayFrame.dtype)


        difference = grayFrame - oldframe
        print "difference dtype:" + str(difference.dtype)
        #difference.dtype = np.float32

        thresholded = difference.copy()

        #cv2.threshold(grayFrame, threshVal, 1, cv2.THRESH_BINARY, thresholded)


        #print "Difference value: " + str(difference.sum())
        #filtered = cv2.filter2D(grayFrame, -1, kernel)
        #filtered = filtered / rho

        cv2.imshow("original", frame)
        cv2.imshow("blurred", grayFrame)
        cv2.imshow("diff", difference)
        cv2.imshow("thresh", thresholded)
        oldframe = grayFrame.copy()

        s.addPoint(difference.sum())
        s.tick()

        cv2.waitKey(1)


