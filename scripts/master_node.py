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
from Master.Scheduler.scheduler import Scheduler


rospy.init_node('vision_master')

def dict_representer(dumper, data):
    return dumper.represent_dict(data.iteritems())

def dict_constructor(loader, node):
    return collections.OrderedDict(loader.construct_pairs(node))

def load_workspace(req):
    rospack = rospkg.RosPack()
    name = os.path.join(rospack.get_path("ros_vision"), "workspaces/" + req.name + ".yaml")

    if not os.path.exists(name):
        name = req.name + ".yaml"

    workspace.reset()
    workspace.name = req.name

    with open(name, 'r') as f:
        for filtergroup_name, filters in yaml.load(f).items():
            workspace.add_group(filtergroup_name, filters)

    return ros_vision.srv.LoadWorkspaceResponse()

def list_workspaces(req):
    res = ros_vision.srv.ListWorkspacesResponse()
    rospack = rospkg.RosPack()
    res.workspaces = []

    for workspace in os.listdir(os.path.join(rospack.get_path("ros_vision"), "workspaces")):
        if workspace.endswith(".yaml"):
            res.workspaces.append(workspace[:-5])

    return res

def save_filterchain():
    pass

def create_filtergroup(req):
    workspace.add_group(req.name)

def list_filter_types(req):
    res = ros_vision.srv.ListFilterTypesResponse()
    descriptors = Filter.list_descriptors()

    for filter_name in descriptors.keys():
        res.filter_list.filters.append(MessageFactory.create_filter_message_from_descriptor(descriptors[filter_name]))

    return res

def on_workspace_update():
    workspace_publisher.publish(MessageFactory.create_workspace_message_from_workspace(workspace))

workspace = Workspace()

list_workspaces_service = rospy.Service('~list_workspaces', ros_vision.srv.ListWorkspaces, list_workspaces)
load_workspace_service = rospy.Service('~load_workspace', ros_vision.srv.LoadWorkspace, load_workspace)
create_filtergroup_service = rospy.Service('~create_filtergroup', ros_vision.srv.CreateFilterGroup, create_filtergroup)
list_filter_types_service = rospy.Service('~list_filter_types', ros_vision.srv.ListFilterTypes, list_filter_types)
workspace_publisher = rospy.Publisher('~workspace', ros_vision.msg.Workspace, queue_size=1, latch=True)

yaml.add_representer(collections.OrderedDict, dict_representer)
yaml.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, dict_constructor)

# req = ros_vision.srv.LoadWorkspaceRequest()
# req.name = 'test_vert_orange'
# load_workspace(req)

while not workspace.is_ready() and not rospy.is_shutdown():
    rospy.sleep(0.1)

workspace.add_update_listener(on_workspace_update)
on_workspace_update()

scheduler = Scheduler(workspace)
scheduler.run()





