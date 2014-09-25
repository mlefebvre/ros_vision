from io_object import IOObject
import sensor_msgs.msg as m
from cv_bridge import CvBridge, CvBridgeError


class Image(IOObject):
    bridge = CvBridge()

    def to_ros_msg(self):
        return m.Image(self.value)

    @staticmethod
    def from_ros_msg(data):
        return Image(Image.bridge.imgmsg_to_cv2(data, "bgr8"))

    @staticmethod
    def get_ros_type():
        return m.Image


