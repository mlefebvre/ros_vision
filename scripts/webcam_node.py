#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image

rospy.init_node('webcam_node')


bridge = CvBridge()

webcam = cv2.VideoCapture(0)
pub = rospy.Publisher("/capra_camera/image", Image, queue_size=10)

if webcam.isOpened(): # try to get the first frame
    rval, frame = webcam.read()
else:
    rval = False
 
while rval:
    msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    pub.publish(msg)
    rval, frame = webcam.read()
    cv2.imshow("webcam", frame)
    k = cv2.waitKey(10) & 0xFF
    if k == 27 or k == 99:
        rval = False
