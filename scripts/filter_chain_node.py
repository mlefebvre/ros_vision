#!/usr/bin/env python
from samba import descriptor

import roslib; roslib.load_manifest('ros_vision')
import rospy
from RosVision.filter_chain import FilterChain
from RosVision.Filters.filter_factory import FilterFactory
import ros_vision.srv
import ros_vision.msg
from RosVision.io_manager import IOManager

def update_filter_topic():
    msg = ros_vision.msg.FilterList()
    for f in fc.get_filters():
        msg.filters.append(create_filter_message(f))

    filter_topic.publish(msg)

def create_base_filter_message(descriptor):
    filter = ros_vision.msg.Filter()
    filter.name = descriptor.get_name()
    filter.description = descriptor.description

    for i in descriptor.get_inputs():
        d = ros_vision.msg.IODescriptor()
        name = i.get_name()
        d.name = name
        d.type = str(i.get_io_type().get_ros_type()._type)
        d.topic = IOManager().format_topic_name(filter.name + "/" + name)
        filter.inputs.append(d)

    for o in descriptor.get_outputs():
        d = ros_vision.msg.IODescriptor()
        name = o.get_name()
        d.name = name
        d.type = str(o.get_io_type().get_ros_type()._type)
        d.topic = IOManager().format_topic_name(filter.name + "/" + name)
        filter.outputs.append(d)

    for p in descriptor.get_parameters():
        parameter = ros_vision.msg.Parameter()
        parameter.name = p.get_name()
        parameter.description = p.get_description()
        parameter.type = p.get_type().__name__
        parameter.default = str(p.get_default_value())
        parameter.min = str(p.get_min_value())
        parameter.max = str(p.get_max_value())
        filter.parameters.append(parameter)

    return filter

def create_filter_message(f):
        filter = create_base_filter_message(f.descriptor)
        filter.name = f.name

        for idx, i in enumerate(f.descriptor.get_inputs()):
            filter.inputs[idx].topic = IOManager().format_topic_name(f.get_io_name(filter.inputs[idx].name))

        for idx, o in enumerate(f.descriptor.get_outputs()):
            filter.outputs[idx].topic = IOManager().format_topic_name(f.get_io_name(filter.outputs[idx].name))

        return filter

def create_filter(req):
    params = {}
    prefix = "%s/%s/" % (rospy.get_name(), req.name)
    for p in rospy.get_param_names():
        if p.startswith(prefix):
            params[p[len(prefix):]] = rospy.get_param(p)

    fc.create_filter(req.name, req.type, params)
    update_filter_topic()

    return ros_vision.srv.CreateFilterResponse()

def list_filters(req):
    res = ros_vision.srv.ListFiltersResponse()

    for d in FilterFactory.list_descriptors():
        res.filter_list.filters.append(create_base_filter_message(d))

    return res

rospy.init_node('filter_chain_node')

fc = FilterChain()

create_filter_service = rospy.Service('~create_filter', ros_vision.srv.CreateFilter, create_filter)
list_filters_service = rospy.Service('~list_filters', ros_vision.srv.ListFilters, list_filters)
filter_topic = rospy.Publisher('~filters', ros_vision.msg.FilterList, queue_size=1, latch=True)
max_rate = rospy.Rate(30)
io_manager = IOManager()
io_manager.connect_scheduler()
last_loop = 0

while not rospy.is_shutdown():
    while not io_manager.received_signal_since(last_loop) and not rospy.is_shutdown():
        rospy.sleep(1.0/1000)

    if not rospy.is_shutdown():
        last_loop = io_manager.get_last_signal()
        fc.execute()
        max_rate.sleep()

