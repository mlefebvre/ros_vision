from io_object import IOObject
from std_msgs.msg import Header
import abc
import rospy


class IOObjectStamped(IOObject):
    __metaclass__ = abc.ABCMeta
    time = 0
    frame = ""

    def __init__(self, value, time=0, frame=""):
        super(IOObjectStamped, self).__init__(value)
        self.time = time
        self.frame = frame

    def _get_header(self):
        header = Header()
        header.frame_id = self.frame
        header.stamp = rospy.Time.from_sec(self.time)
        return header

    def _set_header(self, header):
        self.frame = header.frame_id
        self.time = header.stamp.to_sec()

    @abc.abstractmethod
    def _from_ros_msg(self, value):
        return

    @abc.abstractmethod
    def _to_ros_msg(value):
        return

    def get_frame(self):
        return self.frame

    def get_time(self):
        return self.time

    def set_frame(self, frame):
        self.frame = frame

    def set_time(self, time):
        self.time = time

