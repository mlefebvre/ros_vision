#!/usr/bin/env python
from rospy.impl.registration import get_topic_manager

import tornado.httpserver
import tornado.ioloop
import tornado.web
import tornado.websocket

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import cv2
import os.path
import json
import base64

class RosCvm_Gui(tornado.web.Application):
    def __init__(self):
        handlers = [
            (r"/", DashBoardHandler),
            (r"/input([0-9]+)/", WebsocketHandler),
            (r"/input([0-9]+)/set/", WebsocketHandler),
            (r"/inputs/list/", InputListHandler),
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

class InputListHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        self.write_message(json.dumps(['(None)'] + [topic[0] for topic in rospy.get_published_topics() if topic[1] in valid_topic_types]))

class SetInputHandler(tornado.websocket.WebSocketHandler):
    def on_message(self, message):
        print message

class WebsocketHandler(tornado.websocket.WebSocketHandler):
    def open(self, input_id):
        rospy.Subscriber('/capra_camera/image', Image, self.on_image)

    def on_image(self, image):
        img = CvBridge().imgmsg_to_cv2(image)
        #gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.write_message(base64.encodestring(cv2.imencode('.jpg', img)[1]))

if __name__ == "__main__":
    rospy.init_node('roscvm_gui')

    valid_topic_types = ['sensor_msgs/Image', 'sensor_msgs/PointCloud2']
    feed_inputs = {'feed1': None, 'feed2': None}

    http_server = tornado.httpserver.HTTPServer(RosCvm_Gui())
    http_server.listen(8888)
    tornado.ioloop.IOLoop.instance().start()