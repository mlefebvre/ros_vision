#!/usr/bin/env python

import roslib; roslib.load_manifest('ros_vision')
import rospy
import rospkg
import yaml
import os
from RosVision.filter_chain import FilterChain


def load_yaml(file_name):
    rospack = rospkg.RosPack()
    name = os.path.join(rospack.get_path("ros_vision"), file_name)
    if not os.path.exists(name):
        name = file_name

    with open(name, 'r') as f:
        return yaml.load(f)


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

while not rospy.is_shutdown():
    fc.execute()

