#!/usr/bin/env python
import roslib

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

    fc.create_filter(req.name, req.type, req.order, params)
    update_filter_topic()

    return ros_vision.srv.CreateFilterResponse()


def delete_filter(req):
    fc.delete_filter(req.name)
    update_filter_topic()

    return ros_vision.srv.DeleteFilterResponse()

def set_parameter(req):
    fc.set_param(req.filter_name + "/" + req.parameter_name, req.parameter_value)

    if req.update_topics:
        update_filter_topic()

    return ros_vision.srv.SetParameterValueResponse()


def get_parameter(req):
    res = ros_vision.srv.GetParameterValueResponse()
    res.parameter_value = str(fc.get_param(req.filter_name + "/" + req.parameter_name))

    return res

rospy.init_node('filter_chain_node')

fc = FilterChain()

create_filter_service = rospy.Service('~create_filter', ros_vision.srv.CreateFilter, create_filter)
delete_filter_service = rospy.Service('~delete_filter', ros_vision.srv.DeleteFilter, delete_filter)
set_parameter_service = rospy.Service('~set_parameter', ros_vision.srv.SetParameterValue, set_parameter)
get_parameter_service = rospy.Service('~get_parameter', ros_vision.srv.GetParameterValue, get_parameter)

filter_topic = rospy.Publisher('~filters', ros_vision.msg.FilterList, queue_size=1, latch=True)
stats_topic = rospy.Publisher('~stats', ros_vision.msg.Statistics, queue_size=5)

max_rate = rospy.Rate(30)
io_manager = IOManager()
io_manager.run()
total = 0
count = 0

while not rospy.is_shutdown():
    last = rospy.get_time()
    last_stats = fc.execute()
    avg_stats = fc.get_average_filter_execution_time()
    if len(avg_stats) > 0:
        stat_msg = ros_vision.msg.Statistics()
        for name in avg_stats.keys():
            stat_msg.filters.append(ros_vision.msg.FilterStatistics(name=name, average_execution_time=avg_stats[name], last_execution_time=last_stats[name]))
        stat_msg.average_execution_time = sum(avg_stats.values())
        stats_topic.publish(stat_msg)

    max_rate.sleep()

