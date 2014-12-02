#!/usr/bin/env python

import rospy
from std_msgs.msg._String import String

def callback(msg):
    global pub
    m = String()
    m.data = "A"*100
    print "Pub"
    pub.publish(m)

rospy.init_node('t2')
rospy.Subscriber('/p1', String, callback)
pub = rospy.Publisher('/p2', String, queue_size=1)

t1 = rospy.get_time()

while not rospy.is_shutdown():
    pass
