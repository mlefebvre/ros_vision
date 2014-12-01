#!/usr/bin/env python

import rospy
from std_msgs.msg._String import String

total = 0
nb = 0

def callback(msg):
    global total, nb, t1
    nb += 1
    total += (rospy.get_time() - t1)
    print (total / nb) * 1000

rospy.init_node('t1')
pub = rospy.Publisher('/p1', String, queue_size=1)
rospy.Subscriber('/p2', String, callback)

while not rospy.is_shutdown():
    t1 = rospy.get_time()
    msg = String()
    msg.data = "A" * 100
    pub.publish(msg)
    rospy.sleep(30.0/1000)

