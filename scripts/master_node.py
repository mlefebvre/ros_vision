#!/usr/bin/env python
import importlib
import pkgutil

import roslib;

roslib.load_manifest('ros_vision')
import rospy
import rospkg
import os
import yaml
from Master.filter_chain_node_wrapper import FilterChainNodeWrapper
from Master.scheduler import Scheduler
from RosVision.io_manager import IOManager
from RosVision.Filters.filter import Filter
import ros_vision.srv

rospy.init_node('vision_master')

def load_filterchain(req):
    rospack = rospkg.RosPack()
    name = os.path.join(rospack.get_path("ros_vision"), "configs/" + req.name + ".yaml")

    for name in nodes:
        nodes[name].kill()
        del nodes[name]

    if not os.path.exists(name):
        name = req.name + ".yaml"

    with open(name, 'r') as f:
        for name, params in yaml.load(f).items():
            create_filter_chain_group(name, params)

        scheduler = Scheduler()

        for name in nodes.keys():
            scheduler.add_filter_chain_group(name)

        scheduler.run()

def list_filterchains(req):
    res = ros_vision.srv.ListFilterChainsResponse()
    rospack = rospkg.RosPack()
    res.filterchains = []

    for filterchain in os.listdir(os.path.join(rospack.get_path("ros_vision"), "configs")):
        if filterchain.endswith(".yaml"):
            res.filterchains.append(filterchain[:-5])

    return res

def save_filterchain():
    pass

def create_filtergroup(req):
    f = FilterChainNodeWrapper(req.name, req.properties)
    nodes[req.name] = f

def list_filters(req):
    res = ros_vision.srv.ListFiltersResponse()
    pkgpath = os.path.dirname(os.path.realpath(__file__))

    for _, module, ispkg in pkgutil.iter_modules([pkgpath]):
        if ispkg:
            i = importlib.import_module("RosVision.Filters.%s.filter" % module)
            if hasattr(i, "__dict__"):
                for n, c in i.__dict__.items():
                    try:
                        if issubclass(c, Filter) and n is not "Filter":
                            res.filter_list.filters.append(IOManager.create_base_filter_message(c.descriptor))
                    except:
                        pass

    return res

nodes = {}
list_filterchains_service = rospy.Service('~list_filterchains', ros_vision.srv.ListFilterChains, list_filterchains)
load_filterchain_service = rospy.Service('~load_filterchain', ros_vision.srv.LoadFilterChain, load_filterchain)
create_filtergroup_service = rospy.Service('~create_filtergroup', ros_vision.srv.CreateFilterGroup, create_filtergroup)
list_filters_service = rospy.Service('~list_filters', ros_vision.srv.ListFilters, list_filters)

while not rospy.is_shutdown():
    rospy.Rate(30).sleep()




