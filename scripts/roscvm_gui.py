#!/usr/bin/env python

import tornado.httpserver
import tornado.ioloop
import tornado.web
import tornado.websocket

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

from cv_bridge import CvBridge
import cv2
import os.path
import base64
import json
import datetime

class RosCvm_Gui(tornado.web.Application):
    def __init__(self):
        handlers = [
            (r"/", DashBoardHandler),
            (r"/styles/(.*)", tornado.web.StaticFileHandler, {"path": "assets/css"}),
            (r"/scripts/(.*)", tornado.web.StaticFileHandler, {"path": "assets/js"}),
            (r"/images/(.*)", tornado.web.StaticFileHandler, {"path": "assets/img"}),
            (r"/input([0-9]+)/", SetInputHandler),
            (r"/topiclist/", TopicListHandler),
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

class SetInputHandler(tornado.websocket.WebSocketHandler):
    def open(self, topic):
        #rospy.Subscriber("/capra_camera/image", Image, self.on_image)
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

class TopicListHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        self.send_topics()

    def send_topics(self):
        print dict(filter(lambda topic: topic[0] in input_topic_types, [[topic[1], [topic[0]]] for topic in rospy.get_published_topics()]))
        self.write_message(json.dumps([dict(filter(lambda topic: topic[0] in input_topic_types, [[topic[1], [topic[0]]] for topic in rospy.get_published_topics()]))]))
        tornado.ioloop.IOLoop.instance().add_timeout(input_topic_refresh_rate, self.send_topics)


if __name__ == "__main__":
    rospy.init_node('roscvm_gui')

    input_topic_types = rospy.get_param('~input_topic_types', {'sensor_msgs/Image' : Image, 'sensor_msgs/PointCloud2': PointCloud2})
    input_topic_refresh_rate = datetime.timedelta(seconds=rospy.get_param('~input_topic_refresh_rate', 1))

    http_server = tornado.httpserver.HTTPServer(RosCvm_Gui())
    http_server.listen(8888)
    tornado.ioloop.IOLoop.instance().start()