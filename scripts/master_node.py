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

def get_workspace(req):
    res = ros_vision.srv.GetWorkspaceResponse();
    res.workspace = MessageFactory.create_workspace_message_from_workspace(workspace)

    return res

def load_workspace(req):
    res = ros_vision.srv.LoadWorkspaceResponse();
    rospack = rospkg.RosPack()
    name = os.path.join(rospack.get_path("ros_vision"), "workspaces/" + req.name + ".yaml")

    if not os.path.exists(name):
        name = req.name + ".yaml"

    workspace.reset()
    workspace.name = req.name

    with open(name, 'r') as f:
        for filtergroup_name, filters in yaml.load(f).items():
            workspace.add_group(filtergroup_name, filters)

    while not workspace.is_ready() and not rospy.is_shutdown():
        rospy.sleep(0.1)

    res.workspace = MessageFactory.create_workspace_message_from_workspace(workspace)

    return res

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

def delete_filtergroup(req):
    delete_filter_srv_name = '/%s/delete_filter' % req.filter_group_name
    rospy.wait_for_service(delete_filter_srv_name)
    delete_filter = rospy.ServiceProxy(delete_filter_srv_name, ros_vision.srv.DeleteFilter)

    delete_filter_request = ros_vision.srv.DeleteFilterRequest()
    delete_filter_request.filter_group_name = req.filter_group_name

    for filter_name in workspace.groups[req.filter_group_name].filters.keys():
        delete_filter_request.filter_name = filter_name
        delete_filter(delete_filter_request)

    workspace.groups[req.filter_group_name].kill()
    del workspace.groups[req.filter_group_name]
    workspace.update_workspace()

def list_filter_types(req):
    res = ros_vision.srv.ListFilterTypesResponse()
    descriptors = Filter.list_descriptors()

    for filter_name in descriptors.keys():
        res.filter_list.filters.append(MessageFactory.create_filter_message_from_descriptor(descriptors[filter_name]))

    return res

workspace = Workspace()
#scheduler = Scheduler(workspace)

get_workspace_service = rospy.Service('~get_workspace', ros_vision.srv.GetWorkspace, get_workspace)
list_workspaces_service = rospy.Service('~list_workspaces', ros_vision.srv.ListWorkspaces, list_workspaces)
load_workspace_service = rospy.Service('~load_workspace', ros_vision.srv.LoadWorkspace, load_workspace)
create_filtergroup_service = rospy.Service('~create_filtergroup', ros_vision.srv.CreateFilterGroup, create_filtergroup)
delete_filtergroup_service = rospy.Service('~delete_filtergroup', ros_vision.srv.DeleteFilterGroup, delete_filtergroup)
list_filter_types_service = rospy.Service('~list_filter_types', ros_vision.srv.ListFilterTypes, list_filter_types)

yaml.add_representer(collections.OrderedDict, dict_representer)
yaml.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, dict_constructor)

#scheduler.run()
while not rospy.is_shutdown():
    rospy.sleep(1)
