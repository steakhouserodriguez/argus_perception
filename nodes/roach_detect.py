#!/usr/bin/python

import cv2
import cv
import time
import numpy
import wx
import math

if __name__ == '__main__':
    print "Starting..."

    cap = cv2.VideoCapture(1)
    if not cap:
        print "could not capture from camera!"
        exit(1)

    cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, 480)

    retval, frame = cap.read()
    thresholdedImage = frame.copy()

    kernel = numpy.array([
        [1, 1, 0, -1, -1],
        [1, 0, 0, 0, -1],
        [0, 0, 0, 0, 0],
        [-1, 0, 0, 0, 1],
        [-1, -1, 0, 1, 1]],
                         numpy.float32)


    while True:
        retval, frame = cap.read()
        grayFrame = frame.copy()
        grayFrame = cv2.cvtColor(grayFrame, cv2.COLOR_RGB2GRAY)

        filtered = cv2.filter2D(grayFrame, -1, kernel)
        #filtered = filtered / rho


        cv2.imshow("original", frame)
        #cv2.imshow("sobelH", sobelHImage)
        cv2.imshow("filtered", filtered)
        #cv2.imshow("thresholded", thresholdedImage)
#        cv2.imshow("colored lines", linesColored)
        cv2.waitKey(1)


