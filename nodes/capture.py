#!/usr/bin/env python
'''
Captures files directly from camera to png files

'''
import cv
import cv2


if __name__ == "__main__":

    # Parameters
    CAM = 1
    H_SIZE = 640
    V_SIZE = 480
    DISPLAY = True

    # Open camera
    cap = cv2.VideoCapture(CAM)
    cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, H_SIZE)
    cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, V_SIZE)

    fileindex = 0
    cv2.namedWindow("original")
    while True:

        fileindex = fileindex + 1

        # Capture
        retval, frame = cap.read()

        # Saves file
        cv2.imwrite("capture/cap" + str(fileindex) + ".png", frame, )

        # Display
        if DISPLAY:
            cv2.imshow("original", frame)

        if cv2.waitKey(5) == 0x100000 + ord('q'):
            break;

    end = time.time()       # stop timer
    #times = append(times[1:end], end-start)
    print "FPS: ",count/(end-start)

