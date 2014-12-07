import cv2
import numpy as np

#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image

rospy.init_node('calibration_node')


bridge = CvBridge()
perspective = None

corners = []

def onmouse(event, x, y, flags, param):
    global corners
    if event == cv2.EVENT_LBUTTONDOWN:
        print "New corner: (%d, %d)" % (x, y)
        corners.append([x, y])

cv2.namedWindow("Calibrate", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("Calibrate", onmouse)

while not rospy.is_shutdown():
    img = rospy.wait_for_message("/capra_camera/image", Image)
    frame = bridge.imgmsg_to_cv2(img, "passthrough")

    if perspective is None:
        if len(corners) == 4:
            approx = np.array(corners, np.float32)
            h = np.array([[0, 0], [640, 0], [0, 480], [640, 480]], np.float32)
            perspective = cv2.getPerspectiveTransform(approx, h)
            print perspective
    else:
        frame = cv2.warpPerspective(frame, perspective, (707, 500))

    cv2.imshow("Calibrate", frame)
    # if FIND_PERSPECTIVE:
    #     approx = np.array([[136, 188], [488, 188], [0, 480], [640, 480]], np.float32)
    #     h = np.array([[0, 0], [640, 0], [0, 480], [640, 480]], np.float32)
    #     perspective = cv2.getPerspectiveTransform(approx, h)
    #     print perspective
    # else:
    #     perspective = np.array([[  3.84210526e+00 ,  1.78947368e+00 , -8.58947368e+02],
    #                              [ -3.33066907e-16 ,  6.31578947e+00 , -1.18736842e+03],
    #                              [  2.16840434e-18 ,  5.92105263e-03  , 1.00000000e+00]])
    #     frame = cv2.warpPerspective(frame, perspective, (707, 500))




    k = cv2.waitKey(10) & 0xFF
    if k == 27 or k == 99:
        rval = False
