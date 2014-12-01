#!/usr/bin/env python

import roslib; roslib.load_manifest('ros_vision')
import rospy
import rospkg
import os
import yaml
from Master.filter_chain_node_wrapper import FilterChainNodeWrapper
from Master.scheduler import Scheduler

rospy.init_node('vision_master')


def load_yaml(file_name):
    rospack = rospkg.RosPack()
    name = os.path.join(rospack.get_path("ros_vision"), file_name)
    if not os.path.exists(name):
        name = file_name

    with open(name, 'r') as f:
        return yaml.load(f)

def create_filter_chain_group(name, params):
    f = FilterChainNodeWrapper(name, params)
    nodes[name] = f

nodes = {}

config_file = rospy.get_param("~config", default="configs/test_vert_orange_slow.yaml")
config = load_yaml(config_file)
for name, params in config.items():
    create_filter_chain_group(name, params)

scheduler = Scheduler()

for name in nodes.keys():
    scheduler.add_filter_chain_group(name)

scheduler.run()



