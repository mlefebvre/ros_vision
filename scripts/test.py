import cv2
import numpy as np

#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image

#rospy.init_node('webcam_node')



webcam = cv2.VideoCapture(0)

square_size = 1.0
pattern_size = (9, 6)
pattern_points = np.zeros( (np.prod(pattern_size), 3), np.float32 )
pattern_points[:,:2] = np.indices(pattern_size).T.reshape(-1, 2)
pattern_points *= square_size

objp = np.zeros((6*7,3), np.float32)
objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

if webcam.isOpened(): # try to get the first frame
    rval, frame = webcam.read()
else:
    rval = False

while rval:
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(frame, pattern_size)
    if found:
        term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
        cv2.cornerSubPix(frame, corners, (5, 5), (-1, -1), term)



        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        cv2.drawChessboardCorners(frame, pattern_size, corners, found)

    cv2.imshow("webcam", frame)

    rval, frame = webcam.read()

    k = cv2.waitKey(10) & 0xFF
    if k == 27 or k == 99:
        rval = False
