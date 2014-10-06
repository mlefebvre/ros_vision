from io_object_stamped import IOObjectStamped
import sensor_msgs.msg as m
from cv_bridge import CvBridge, CvBridgeError


class Image(IOObjectStamped):
    bridge = CvBridge()

    def _to_ros_msg(self):
        msg = Image.bridge.cv2_to_imgmsg(self.value, "bgr8")
        msg.header = self._get_header()
        return msg

    def _from_ros_msg(self, data):
        self._set_header(data.header)
        return Image.bridge.imgmsg_to_cv2(data, "bgr8")

    @staticmethod
    def get_ros_type():
        return m.Image


