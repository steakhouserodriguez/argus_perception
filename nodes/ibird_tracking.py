from itertools import izip
import time

from pylab import *
from numpy.random import *
import cv2
import cv

import os

from particle_filter.particle_filter import *

def getMaxIndex(directory):
    files = [f for f in os.listdir(directory) if os.path.isfile(f)]
    max = -1;
    for f in files:
        current = getIndex(f)
        if (current > max):
            max = current
    return max

def getIndex(file):
    if (file.startswith("video")):
        if((file[5]).isdigit):
            indexStr = (((file.split("video"))[1]).split(".avi"))[0]
            return int(indexStr)
    else:
        return 0

def draw_particles(frame, p):
    pos, xs, ws = p
    #print( "position: (" + str(pos[1]) + "," + str(pos[0]) +  ")")
    cv2.circle(frame, (pos[1], pos[0]), 10, (255, 0, 0))    # Draw position estimate
    frame[tuple(xs.T)] = (0, 0, 255)                        # Draw particles
    return frame

def pf_test(n_frames=200, n_particles=1000, h_size=640, v_size=480, display=False):

    # matplotlib interactive mode
    ion()

    # Create an image sequence of 20 frames long
    seq = [im for im in zeros((N_FRAMES,V_SIZE,H_SIZE,3), int)]

    # Add a square with starting position x0 moving along trajectory xs
    x0 = array([120, 160])
    xs = vstack((arange(N_FRAMES)*3, arange(N_FRAMES)*2)).T + x0
    for t, x in enumerate(xs):
        xslice = slice(x[0]-8, x[0]+8)
        yslice = slice(x[1]-8, x[1]+8)
        seq[t][xslice, yslice] = 255

    # Initialize particle filter
    p_init = randint(0, V_SIZE, (N_PARTICLES, 2))
    color_model = iter(seq).next()[tuple(x0)]
    pf = ParticleFilter(p_init, 8, color_model, seq[0].shape, N_PARTICLES)

    # Run particle filter
    sequence = iter(seq)
    start = time.time()
    for im in sequence:
        pf.elapse_time()
        pf.observe(im)
        if display:
            draw_particles(im, pf.get_state())
    end = time.time()
    print "FPS: ",N_FRAMES/(end-start)

if __name__ == "__main__":

    # Parameters
    N_PARTICLES = 1000
    CAM = 1
    H_SIZE = 640
    V_SIZE = 480
    DISPLAY = True
    LIVE = True
    RECORD = True

    if LIVE:
        # Open camera
        cap = cv2.VideoCapture(CAM)
        cap.set(cv.CV_CAP_PROP_FRAME_WIDTH, H_SIZE)
        cap.set(cv.CV_CAP_PROP_FRAME_HEIGHT, V_SIZE)
        retval, frame = cap.read()


    if RECORD:
        file1 = "video" + str(getMaxIndex('.') + 1) + ".avi"
        writer1 = cv2.VideoWriter(
            file1,
            cv.CV_FOURCC('U','2','6','3'),
            #0,
            30,
            (H_SIZE, V_SIZE))

        if(not writer1.isOpened()):
            print("VideoWriter failed to open!")
        else:
            print("saving to file1=" +file1)

            file2 = "video" + str(getMaxIndex('.') + 1) + ".avi"
            writer2 = cv2.VideoWriter(
                file2,
                cv.CV_FOURCC('U','2','6','3'),
                #0,
                30,
                (H_SIZE, V_SIZE))

            if(not writer2.isOpened()):
                print("VideoWriter failed to open!")
            else:
                print("saving to file2=" +file2)


        #writer.open("capture/video", cv.CV_FOURCC('F','L','V','1'), 30, (H_SIZE, V_SIZE), 1)

    # Initialize particle filter
    p_init = randint(0, V_SIZE, (N_PARTICLES, 2))
    pf = ParticleFilter(p_init, 20, frame.shape[:-1], N_PARTICLES)

    # Run particle filter
    times = 30*ones(10)
    count = 0
    start = time.time()     # start timer


    if LIVE:
        retval, grayFrame = cap.read()

    oldFrame = cv2.cvtColor(grayFrame, cv2.COLOR_RGB2GRAY, grayFrame)


    cv2.namedWindow("original")
    cv2.namedWindow("motion")

    while True:
        count = count + 1

        # Capture
        retval, frame = cap.read()

        # Converts image to grayscale
        grayFrame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        grayFrame = cv2.GaussianBlur(grayFrame, (9,9), 0)

        grayFrame = grayFrame / 255.0

        # Image subtraction to detect motion
        difference = grayFrame - oldFrame

        # Process Particle filter
        pf.elapse_time()
        pf.observe(difference)

        # Saves old grayFrame for next tick
        oldFrame = grayFrame.copy()

        # Converts image back to color for particle vis
#        difference = difference * 10.0
        display = difference * 10.0
        display = display.astype(uint8)

        display = cv2.cvtColor(display, cv2.COLOR_GRAY2BGR)

        # draw particles
        display = draw_particles(display, pf.get_state())
        frame = draw_particles(frame, pf.get_state())

        # Display
        if DISPLAY:
            cv2.imshow("original", frame)
            cv2.imshow("motion", display)

        if RECORD:
            writer1.write(frame)
            writer2.write(display)

        if cv2.waitKey(5) ==  ord('q'):
            break;

    end = time.time()       # stop timer
    #times = append(times[1:end], end-start)
    print "FPS: ",count/(end-start)

