#!/usr/bin/env python

import roslib; roslib.load_manifest('ros_vision')
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image

rospy.init_node('image_publisher_node')

argv = rospy.myargv()
image = argv[1]
topic = argv[2]

im = cv2.imread(image)
bridge = cv_bridge.CvBridge()
msg = bridge.cv2_to_imgmsg(im, encoding="bgr8")
pub = rospy.Publisher(topic, Image, queue_size=10)

while not rospy.is_shutdown():
    msg.header.stamp = rospy.get_rostime()
    pub.publish(msg)
    rospy.sleep(1)
