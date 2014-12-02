#!/usr/bin/env python
from samba import descriptor

import roslib;

roslib.load_manifest('ros_vision')
import rospy
import ros_vision.srv
import ros_vision.msg
from RosVision.filter_chain import FilterChain
from RosVision.io_manager import IOManager
from RosVision.message_factory import MessageFactory

def update_filter_topic():
    msg = ros_vision.msg.FilterList()
    for f in fc.get_filters():
        msg.filters.append(MessageFactory.create_filter_message_from_filter(f))

    filter_topic.publish(msg)

def create_filter(req):
    params = {}
    prefix = "%s/%s/" % (rospy.get_name(), req.name)
    for p in rospy.get_param_names():
        if p.startswith(prefix):
            params[p[len(prefix):]] = rospy.get_param(p)

    fc.create_filter(req.name, req.type, params)
    update_filter_topic()

    return ros_vision.srv.CreateFilterResponse()

rospy.init_node('filter_chain_node')

fc = FilterChain()

create_filter_service = rospy.Service('~create_filter', ros_vision.srv.CreateFilter, create_filter)
filter_topic = rospy.Publisher('~filters', ros_vision.msg.FilterList, queue_size=1, latch=True)
max_rate = rospy.Rate(30)
io_manager = IOManager()
io_manager.run()
last_loop = 0

while not rospy.is_shutdown():
    while not io_manager.received_signal_since(last_loop) and not rospy.is_shutdown():
        rospy.sleep(1.0/1000)

    if not rospy.is_shutdown():
        last_loop = io_manager.get_last_signal()
        fc.execute()
        max_rate.sleep()

