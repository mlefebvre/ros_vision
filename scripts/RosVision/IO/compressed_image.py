from io_object_stamped import IOObjectStamped
import sensor_msgs.msg as m
import numpy as np
import cv2


class CompressedImage(IOObjectStamped):
    def _to_ros_msg(self):
        msg = m.CompressedImage()
        msg.header = self._get_header()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', self.value)[1]).tostring()
        return msg

    def _from_ros_msg(self, msg):
        self._set_header(msg.header)
        np_arr = np.fromstring(msg.data, np.uint8)
        value = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        return value

    @staticmethod
    def get_ros_type():
        return m.CompressedImage

    def get_image(self):
        if type(self.value) == self.get_ros_type():
            self.value = self._from_ros_msg(self.value)
        return self.value


