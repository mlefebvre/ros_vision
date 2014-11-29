#!/usr/bin/env python
import roslib;

roslib.load_manifest('ros_vision')
import rospy
import rospkg
import os
import yaml
import collections
import importlib
import pkgutil
from Master.filter_chain_node_wrapper import FilterChainNodeWrapper
from Master.scheduler import Scheduler
from RosVision.io_manager import IOManager
from RosVision.Filters.filter import Filter
import ros_vision.srv
import ros_vision.msg

rospy.init_node('vision_master')

def dict_representer(dumper, data):
    return dumper.represent_dict(data.iteritems())

def dict_constructor(loader, node):
    return collections.OrderedDict(loader.construct_pairs(node))

def list_descriptors():
    descriptors = {}
    pkgpath = os.path.dirname(os.path.realpath(__file__))

    for _, module, ispkg in pkgutil.iter_modules([pkgpath]):
        if ispkg:
            i = importlib.import_module("RosVision.Filters.%s.filter" % module)
            if hasattr(i, "__dict__"):
                for n, c in i.__dict__.items():
                    try:
                        if issubclass(c, Filter) and n is not "Filter":
                            descriptors[n] = c.descriptor
                    except:
                        pass

    return descriptors

def load_filterchain(req):
    rospack = rospkg.RosPack()
    name = os.path.join(rospack.get_path("ros_vision"), "configs/" + req.name + ".yaml")

    for name in nodes:
        nodes[name].kill()
        del nodes[name]

    if not os.path.exists(name):
        name = req.name + ".yaml"

    with open(name, 'r') as f:
        for filtergroup_name, filters in yaml.load(f).items():
            f = FilterChainNodeWrapper(req.name, filters)
            nodes[req.name] = f

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
    f = FilterChainNodeWrapper(req.name)
    nodes[req.name] = f

    return ros_vision.srv.CreateFilterGroupResponse()

def list_filters(req):
    res = ros_vision.srv.ListFiltersResponse()

    for name, descriptor in list_descriptors():
        res.filter_list.filters.append(IOManager.create_filter_message_from_descriptor(descriptor))

    return res

nodes = {}
list_filterchains_service = rospy.Service('~list_filterchains', ros_vision.srv.ListFilterChains, list_filterchains)
load_filterchain_service = rospy.Service('~load_filterchain', ros_vision.srv.LoadFilterChain, load_filterchain)
create_filtergroup_service = rospy.Service('~create_filtergroup', ros_vision.srv.CreateFilterGroup, create_filtergroup)
list_filters_service = rospy.Service('~list_filters', ros_vision.srv.ListFilters, list_filters)

yaml.add_representer(collections.OrderedDict, dict_representer)
yaml.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, dict_constructor)

while not rospy.is_shutdown():
    rospy.Rate(30).sleep()




