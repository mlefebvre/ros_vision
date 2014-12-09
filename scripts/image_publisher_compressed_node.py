#!/usr/bin/env python

import roslib; roslib.load_manifest('ros_vision')
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import CompressedImage
import numpy as np


rospy.init_node('image_publisher_node')

argv = rospy.myargv()
image = argv[1]
topic = argv[2]

im = cv2.imread(image)

msg = CompressedImage()
msg.format = "jpeg"
msg.data = np.array(cv2.imencode('.jpg', im)[1]).tostring()

pub = rospy.Publisher(topic, CompressedImage, queue_size=10)

while not rospy.is_shutdown():
    msg.header.stamp = rospy.get_rostime()
    pub.publish(msg)
    rospy.sleep(0.1)
