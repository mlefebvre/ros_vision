#!/usr/bin/env python

import tornado.httpserver
import tornado.ioloop
import tornado.web
import tornado.websocket

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from ros_vision.msg import FilterGroupList
from ros_vision.msg import FilterList
import ros_vision.srv

from cv_bridge import CvBridge
import cv2
import os.path
import base64
import jsonpickle as json
import datetime

class MasterHandler(tornado.websocket.WebSocketHandler):
    _filterchain = {}
    _connected_clients = []

    @staticmethod
    def get_filterchain():
        return MasterHandler._filterchain

    @staticmethod
    def update_filtergroup_informations(filtergroups):
        for filtergroup in filtergroups.filtergroups:
            if filtergroup.name not in MasterHandler._filterchain:
                MasterHandler._filterchain[filtergroup.name] = {}

            MasterHandler._filterchain[filtergroup.name]["informations"] = [
                rospy.Subscriber("/" + filtergroup.name + filtergroup.filters_publisher, FilterList, MasterHandler.update_filtergroup_filters, callback_args=filtergroup.name),
                rospy.ServiceProxy("/" + filtergroup.name + filtergroup.delete_filtergroup_service, ros_vision.srv.DeleteFilterGroup),
                #rospy.ServiceProxy("/" + filtergroup.name + filtergroup.update_filtergroup_service, ros_vision.srv.UpdateFilterGroup),
                rospy.ServiceProxy("/" + filtergroup.name + filtergroup.create_filter_service, ros_vision.srv.CreateFilter),
                rospy.ServiceProxy("/" + filtergroup.name + filtergroup.delete_filter_service, ros_vision.srv.DeleteFilter)
                #rospy.ServiceProxy("/" + filtergroup.name + filtergroup.update_filter_service, ros_vision.srv.UpdateFilter)
            ]

    @staticmethod
    def update_filtergroup_filters(filter_list, filtergroup_name):
        MasterHandler._filterchain[filtergroup_name]["filters"] = []

        for filter in filter_list.filters:
            f = {}
            f["name"] = filter.name
            f["type"] = filter.type
            f["inputs"] = []
            f["outputs"] = []
            f["parameters"] = []

            for i in filter.inputs:
                f["inputs"].append({
                    "name": i.name,
                    "type": i.type,
                    "topic" : i.topic
                })

            for o in filter.outputs:
                f["outputs"].append({
                    "name": o.name,
                    "type": o.type,
                    "topic" : o.topic
                })

            for p in filter.parameters:
                f["parameters"].append({
                    "name": p.name,
                    "type": p.type,
                    "description" : p.description,
                    "min" : p.min,
                    "max" : p.max,
                    "default" : p.default,
                    "value" : p.default
                })

            MasterHandler._filterchain[filtergroup_name]["filters"].append(f)

        for client in MasterHandler._connected_clients:
            client.update(filtergroup_name)

    @staticmethod
    def delete_filtergroup(filtergroup_name):
        del MasterHandler._filterchain[filtergroup_name]

    @staticmethod
    def delete_filter(filtergroup_name, filter_name):
        pass

    @staticmethod
    def add_filter(filtergroup_name, filter):
        pass

    def open(self):
        MasterHandler._connected_clients.append(self)

        for filtergroup_name in MasterHandler._filterchain.keys():
            self.update(filtergroup_name)

    def on_close(self):
        MasterHandler._connected_clients.remove(self)

    def on_message(self, name):
        rospy.wait_for_service('/vision_master/create_filtergroup')

        req = ros_vision.srv.CreateFilterGroupRequest()
        req.name = name
        create_filtergroup = rospy.ServiceProxy('/vision_master/create_filtergroup', ros_vision.srv.CreateFilterGroup)

        create_filtergroup(req)

    def update(self, filtergroup_name):
        self.write_message(json.encode({filtergroup_name : MasterHandler._filterchain[filtergroup_name]["filters"]}, unpicklable=False))

class GUI(tornado.web.Application):
    def __init__(self):
        handlers = [
            (r"/", DashBoardHandler),
            (r"/styles/(.*)", tornado.web.StaticFileHandler, {"path": "assets/css"}),
            (r"/scripts/(.*)", tornado.web.StaticFileHandler, {"path": "assets/js"}),
            (r"/images/(.*)", tornado.web.StaticFileHandler, {"path": "assets/img"}),
            (r"/input([0-9]+)/", InputsHandler),
            (r"/topics/", TopicsHandler),
            (r"/filters/", FiltersHandler),
            (r"/master/", MasterHandler),
            (r"/load/", LoadHandler)
        ]
        settings = dict(
            title=u"ROS CVM",
            static_path=os.path.join(os.path.dirname(__file__), "assets"),
            template_path=os.path.join(os.path.dirname(__file__), "views"),
            xsrf_cookies=True,
            debug=True,
        )
        tornado.web.Application.__init__(self, handlers, **settings)

class DashBoardHandler(tornado.web.RequestHandler):
    def get(self):
        self.render('dashboard.html')

class InputsHandler(tornado.websocket.WebSocketHandler):
    def open(self, topic):
        self.input_subscriber = None

    def on_message(self, message):
        topic, type = message.split(',')

        if self.input_subscriber is not None:
            self.input_subscriber.unregister()

        if type in input_topic_types:
            self.input_subscriber = rospy.Subscriber(topic, input_topic_types[type], self.on_image)
        else:
            self.write_message(type)

    def on_image(self, image):
        img = CvBridge().imgmsg_to_cv2(image)
        #gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if self.ws_connection is not None:
            self.write_message(base64.encodestring(cv2.imencode('.jpg', img)[1]))

class TopicsHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        self.send_topics()

    def send_topics(self):
        filtered_topics = {}
        for value, key in rospy.get_published_topics():
            if key in input_topic_types:
                filtered_topics.setdefault(key, []).append(value)

        self.write_message(json.encode(filtered_topics, unpicklable=False))
        tornado.ioloop.IOLoop.instance().add_timeout(client_refresh_rate, self.send_topics)

class FiltersHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        rospy.wait_for_service('/vision_master/list_filter_types')
        list_filters = rospy.ServiceProxy('/vision_master/list_filter_types', ros_vision.srv.ListFilterTypes)
        self.write_message(json.encode(list_filters().filter_list.filters, unpicklable=False))

class LoadHandler(tornado.websocket.WebSocketHandler):
    _loaded_filterchain_name = None

    @staticmethod
    def get_loaded_filterchain():
        return LoadHandler._loaded_filterchain_name

    def open(self):
        rospy.wait_for_service('/vision_master/list_filterchains')
        list_filterchains = rospy.ServiceProxy('/vision_master/list_filterchains', ros_vision.srv.ListFilterChains)
        self.write_message(json.encode(list_filterchains().filterchains, unpicklable=False))

    def on_message(self, message):
        req = ros_vision.srv.LoadFilterChainRequest()
        LoadHandler._loaded_filterchain_name = req.name = message.encode('ascii','ignore')

        rospy.wait_for_service('/vision_master/load_filterchain')
        load_filterchain = rospy.ServiceProxy('/vision_master/load_filterchain', ros_vision.srv.LoadFilterChain)
        load_filterchain(req)

"""class SaveHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        pass"""

rospy.init_node('roscvm_gui')

client_refresh_rate = datetime.timedelta(seconds=rospy.get_param('~input_topic_refresh_rate', 1))
input_topic_types = rospy.get_param('~input_topic_types', {'sensor_msgs/Image' : Image, 'sensor_msgs/PointCloud2': PointCloud2})

rospy.Subscriber('/vision_master/filtergroups', FilterGroupList, MasterHandler.update_filtergroup_informations)

http_server = tornado.httpserver.HTTPServer(GUI())
http_server.listen(8888)
tornado.ioloop.IOLoop.instance().start()