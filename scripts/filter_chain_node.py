#!/usr/bin/env python

import roslib; roslib.load_manifest('ros_vision')
import rospy
import rospkg
import yaml
import os
from RosVision.filter_chain import FilterChain
import ros_vision.srv
import ros_vision.msg

def load_yaml(file_name):
    rospack = rospkg.RosPack()
    name = os.path.join(rospack.get_path("ros_vision"), file_name)
    if not os.path.exists(name):
        name = file_name

    with open(name, 'r') as f:
        return yaml.load(f)

def list_filters(req):
    response = ros_vision.srv.ListFiltersResponse()
    for f in fc.get_filters():
        descriptor = f.get_descriptor()

        filter = ros_vision.msg.Filter()
        filter.name = f.name
        filter.description = descriptor.description

        for i in descriptor.get_inputs():
            d = ros_vision.msg.IODescriptor()
            name = i.get_name()
            d.name = name
            d.topic = f.get_topic_name(name)
            filter.inputs.append(d)

        for o in descriptor.get_outputs():
            d = ros_vision.msg.IODescriptor()
            name = o.get_name()
            d.name = name
            d.topic = f.get_topic_name(name)
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

        response.filters.append(filter)

    return response


rospy.init_node('filter_chain_node')

fc = FilterChain()

# Load configuration
config_file = rospy.get_param("~config", default="configs/default.yaml")
config = load_yaml(config_file)
ros_params = {}

# Load filters
for name, params in sorted(config.items(), key=lambda x: x[1]['order']):
    filter_type = params['type']
    del params['order']
    del params['type']
    for p, v in params.items():
        ros_params["%s/%s" % (name, p)] = v

    fc.create_filter(name, filter_type, params)

# Set ROS parameters for each filter
for param, value in ros_params.items():
    rospy.set_param("~%s" % param, value)

filter_service = rospy.Service('~list_filters', ros_vision.srv.ListFilters, list_filters)

while not rospy.is_shutdown():
    fc.execute()

