#!/usr/bin/env python
import roslib; roslib.load_manifest('ros_vision')

import rospy
import rospkg
import os
import yaml
import collections
from RosVision.message_factory import MessageFactory
from RosVision.Filters.filter import Filter
import ros_vision.srv
import ros_vision.msg
from Master.Workspace.workspace import Workspace

rospy.init_node('vision_master')

def dict_representer(dumper, data):
    return dumper.represent_dict(data.iteritems())

def dict_constructor(loader, node):
    return collections.OrderedDict(loader.construct_pairs(node))

def load_filterchain(req):
    rospack = rospkg.RosPack()
    name = os.path.join(rospack.get_path("ros_vision"), "configs/" + req.name + ".yaml")

    if not os.path.exists(name):
        name = req.name + ".yaml"

    workspace.reset()

    with open(name, 'r') as f:
        for filtergroup_name, filters in yaml.load(f).items():
            workspace.add_group(filtergroup_name, filters)

        filtergroup_list_publisher.publish(MessageFactory.create_filtergrouplist_message_from_string_list(workspace.get_filter_groups_names()))

    return ros_vision.srv.LoadFilterChainResponse()

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
    workspace.add_group(req.name)
    filtergroup_list_publisher.publish(MessageFactory.create_filtergrouplist_message_from_string_list(workspace.get_filter_groups_names()))

    return ros_vision.srv.CreateFilterGroupResponse()

def list_filter_types(req):
    res = ros_vision.srv.ListFilterTypesResponse()
    descriptors = Filter.list_descriptors()

    for filter_name in descriptors.keys():
        res.filter_list.filters.append(MessageFactory.create_filter_message_from_descriptor(descriptors[filter_name]))

    return res

def list_filtergroups(req):
    return MessageFactory.create_filtergrouplist_message_from_string_list(workspace.get_filter_groups_names())

def on_workspace_update():
    print workspace.input_topics
    print workspace.output_topics
    print workspace.filter_chains[0].nodes(data=True)

workspace = Workspace()

list_filterchains_service = rospy.Service('~list_filterchains', ros_vision.srv.ListFilterChains, list_filterchains)
load_filterchain_service = rospy.Service('~load_filterchain', ros_vision.srv.LoadFilterChain, load_filterchain)
create_filtergroup_service = rospy.Service('~create_filtergroup', ros_vision.srv.CreateFilterGroup, create_filtergroup)
list_filtergroup_service = rospy.Service('~list_filtergroups', ros_vision.srv.ListFilterGroups, list_filtergroups)
list_filter_types_service = rospy.Service('~list_filter_types', ros_vision.srv.ListFilterTypes, list_filter_types)

filtergroup_list_publisher = rospy.Publisher('~filtergroups', ros_vision.msg.FilterGroupList, queue_size=1, latch=True)

yaml.add_representer(collections.OrderedDict, dict_representer)
yaml.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, dict_constructor)


req = ros_vision.srv.LoadFilterChainRequest()
req.name = 'test_vert_orange'
load_filterchain(req)

while not workspace.is_ready() and not rospy.is_shutdown():
    rospy.sleep(0.1)

workspace.add_update_listener(on_workspace_update)
on_workspace_update()

while not rospy.is_shutdown():
    rospy.Rate(30).sleep()




