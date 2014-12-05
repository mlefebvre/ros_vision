#!/usr/bin/env python

import tornado.httpserver
import tornado.ioloop
import tornado.web
import tornado.websocket

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CompressedImage
import ros_vision.msg
import ros_vision.srv

from cv_bridge import CvBridge
import cv2
import os.path
import base64
import json
from util.message_encoder import MessageEncoder
import datetime

class MasterHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        self.master_subscriber = rospy.Subscriber('/vision_master/workspace', ros_vision.msg.Workspace, self.update_workspace)

    def on_close(self):
        self.master_subscriber.unregister()

    def on_message(self, name):
        rospy.wait_for_service('/vision_master/create_filtergroup')

        req = ros_vision.srv.CreateFilterGroupRequest()
        req.name = name
        create_filtergroup = rospy.ServiceProxy('/vision_master/create_filtergroup', ros_vision.srv.CreateFilterGroup)

        create_filtergroup(req)

    def update_workspace(self, workspace):
        self.write_message(json.dumps(workspace, cls=MessageEncoder))

class GUI(tornado.web.Application):
    def __init__(self):
        handlers = [
            (r"/", DashBoardHandler),
            (r"/styles/(.*)", tornado.web.StaticFileHandler, {"path": "assets/css"}),
            (r"/scripts/(.*)", tornado.web.StaticFileHandler, {"path": "assets/js"}),
            (r"/images/(.*)", tornado.web.StaticFileHandler, {"path": "assets/img"}),
            (r"/fonts/(.*)", tornado.web.StaticFileHandler, {"path": "assets/fonts"}),
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

        self.write_message(json.dumps(filtered_topics))
        tornado.ioloop.IOLoop.instance().add_timeout(client_refresh_rate, self.send_topics)

class FiltersHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        rospy.wait_for_service('/vision_master/list_filter_types')
        list_filter_types = rospy.ServiceProxy('/vision_master/list_filter_types', ros_vision.srv.ListFilterTypes)
        self.write_message(json.dumps(list_filter_types().filter_list.filters, cls=MessageEncoder))

class LoadHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        rospy.wait_for_service('/vision_master/list_workspaces')
        list_workspaces = rospy.ServiceProxy('/vision_master/list_workspaces', ros_vision.srv.ListWorkspaces)
        self.write_message(json.dumps(list_workspaces().workspaces))

    def on_message(self, message):
        req = ros_vision.srv.LoadWorkspaceRequest()
        req.name = message.encode('ascii','ignore')

        rospy.wait_for_service('/vision_master/load_workspace')
        load_workspace = rospy.ServiceProxy('/vision_master/load_workspace', ros_vision.srv.LoadWorkspace)
        load_workspace(req)

"""class SaveHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        pass"""

rospy.init_node('roscvm_gui')

client_refresh_rate = datetime.timedelta(seconds=rospy.get_param('~input_topic_refresh_rate', 1))
input_topic_types = rospy.get_param('~input_topic_types', {'sensor_msgs/Image' : Image, 'sensor_msgs/PointCloud2': PointCloud2, 'sensor_msgs/CompressedImage' : CompressedImage})

http_server = tornado.httpserver.HTTPServer(GUI())
http_server.listen(8888)
tornado.ioloop.IOLoop.instance().start()