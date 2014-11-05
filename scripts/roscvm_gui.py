#!/usr/bin/env python

import tornado.httpserver
import tornado.ioloop
import tornado.web
import tornado.websocket

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import cv2
import os.path
import base64

class RosCvm_Gui(tornado.web.Application):
    def __init__(self):
        handlers = [
            (r"/", DashBoardHandler),
            (r"/input([0-9]+)/", InputHandler),
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

class InputHandler(tornado.websocket.WebSocketHandler):
    def open(self, input_id):
        rospy.Subscriber('/capra_camera/image', Image, self.on_image)

    def on_image(self, image):
        img = CvBridge().imgmsg_to_cv2(image)
        #gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.write_message(base64.encodestring(cv2.imencode('.jpg', img)[1]))

if __name__ == "__main__":
    rospy.init_node('roscvm_gui')

    http_server = tornado.httpserver.HTTPServer(RosCvm_Gui())
    http_server.listen(8888)
    tornado.ioloop.IOLoop.instance().start()