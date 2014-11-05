#!/usr/bin/env python

import roslib; roslib.load_manifest('ros_vision')
import rospy
import rospkg
import os
import yaml
from Master.filter_chain_node_wrapper import FilterChainNodeWrapper


rospy.init_node('master_node')


def load_yaml(file_name):
    rospack = rospkg.RosPack()
    name = os.path.join(rospack.get_path("ros_vision"), file_name)
    if not os.path.exists(name):
        name = file_name

    with open(name, 'r') as f:
        return yaml.load(f)

nodes = {}

config_file = rospy.get_param("~config", default="configs/test_vert_orange.yaml")#default.yaml")
config = load_yaml(config_file)
for name, params in config.items():
    f = FilterChainNodeWrapper(name, params)
    nodes[name] = f


while not rospy.is_shutdown():
    rospy.sleep(1)




