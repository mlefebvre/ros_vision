#!/usr/bin/env python
import uuid

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
    clients = set()

    def open(self):
        self.id = uuid.uuid4()
        MasterHandler.clients.add(self)
        MasterHandler.notify_all_clients()

    def on_message(self, message):
        data = json.loads(message)

        if "set_parameter" in data:
            parameter = data["set_parameter"]
            service_name = '/%s/set_parameter' % parameter["filter_group_name"]

            req = ros_vision.srv.SetParameterValueRequest()
            req.filter_name = parameter["filter_name"]
            req.parameter_name = parameter["parameter_name"]
            req.parameter_value = str(parameter["parameter_value"])

            rospy.wait_for_service(service_name)
            set_parameter_value = rospy.ServiceProxy(service_name, ros_vision.srv.SetParameterValue)
            set_parameter_value(req)

            MasterHandler.clients.discard(self)
            [client.write_message(json.dumps({
                "parameter":
                    {
                        "id": "#" + parameter["parameter_name"],
                        "value": str(parameter["parameter_value"])
                    }
                })) for client in MasterHandler.clients]
            MasterHandler.clients.add(self)
        elif "get_parameter" in data:
            parameter = data["get_parameter"]
            service_name = '/%s/get_parameter' % parameter["filter_group_name"]

            req = ros_vision.srv.GetParameterValueRequest()
            req.filter_name = parameter["filter_name"]
            req.parameter_name = parameter["parameter_name"]

            rospy.wait_for_service(service_name)
            get_parameter_value = rospy.ServiceProxy(service_name, ros_vision.srv.GetParameterValue)
            loaded_parameter = get_parameter_value(req)

            self.write_message(json.dumps({
                "parameter":
                    {
                        "name": parameter["parameter_name"],
                        "value": loaded_parameter.parameter_value
                    }
                }))
        elif "create_filter" in data:
            filter = data["create_filter"]
            service_name = '/%s/create_filter' % filter["filter_group_name"]

            req = ros_vision.srv.CreateFilterRequest()
            req.name = filter["filter_name"]
            req.type = filter["type"]
            req.order = filter["order"]

            rospy.wait_for_service(service_name)
            create_filter = rospy.ServiceProxy(service_name, ros_vision.srv.CreateFilter)
            create_filter(req)

            MasterHandler.clients.discard(self)
            [client.write_message(json.dumps({
                "filter":
                    {
                        "filter_group_name": filter["filter_group_name"],
                        "filter_name": filter["filter_name"],
                        "type": filter["type"],
                        "order": filter["order"]
                    }
            })) for client in MasterHandler.clients]
            MasterHandler.clients.add(self)

        elif "delete_filter" in data:
            filter = data["delete_filter"]
            service_name = '/%s/delete_filter' % filter["filter_group_name"]

            req = ros_vision.srv.DeleteFilterRequest()
            req.name = filter["filter_name"]

            rospy.wait_for_service(service_name)
            delete_filter = rospy.ServiceProxy(service_name, ros_vision.srv.DeleteFilter)
            delete_filter(req)

            MasterHandler.clients.discard(self)
            [client.write_message(json.dumps({
                "filter":
                    {
                        "filter_group_name": filter["filter_group_name"],
                        "filter_name": filter["filter_name"]
                    }
            })) for client in MasterHandler.clients]
            MasterHandler.clients.add(self)
        elif "create_filter_group" in data:
            pass
        elif "delete_filter_group" in data:
            pass

    def on_close(self):
        MasterHandler.clients.remove(self)

    @staticmethod
    def notify_all_clients(workspace=None):
        if workspace is None:
            rospy.wait_for_service('/vision_master/get_workspace')
            get_workspace = rospy.ServiceProxy('/vision_master/get_workspace', ros_vision.srv.GetWorkspace)
            workspace = get_workspace().workspace

        [client.write_message(json.dumps({"workspace" : workspace}, cls=MessageEncoder)) for client in MasterHandler.clients]

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
    clients = set()

    def open(self):
        LoadHandler.clients.add(self)
        rospy.wait_for_service('/vision_master/list_workspaces')
        list_workspaces = rospy.ServiceProxy('/vision_master/list_workspaces', ros_vision.srv.ListWorkspaces)
        self.write_message(json.dumps(list_workspaces().workspaces))

    def on_message(self, message):
        req = ros_vision.srv.LoadWorkspaceRequest()
        req.name = message.encode('ascii','ignore')

        rospy.wait_for_service('/vision_master/load_workspace')
        load_workspace = rospy.ServiceProxy('/vision_master/load_workspace', ros_vision.srv.LoadWorkspace)
        MasterHandler.notify_all_clients(load_workspace(req).workspace)

    def on_close(self):
        LoadHandler.clients.remove(self)

"""class SaveHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        pass"""

rospy.init_node('roscvm_gui')

client_refresh_rate = datetime.timedelta(seconds=rospy.get_param('~input_topic_refresh_rate', 1))
input_topic_types = rospy.get_param('~input_topic_types', {'sensor_msgs/Image' : Image, 'sensor_msgs/PointCloud2': PointCloud2, 'sensor_msgs/CompressedImage' : CompressedImage})

http_server = tornado.httpserver.HTTPServer(GUI())
http_server.listen(8888)
tornado.ioloop.IOLoop.instance().start()