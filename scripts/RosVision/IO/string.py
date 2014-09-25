from io_object import IOObject
import std_msgs.msg as m


class String(IOObject):
    def to_ros_msg(self):
        return m.String(self.value)

    @staticmethod
    def from_ros_msg(data):
        return String(data)

    @staticmethod
    def get_ros_type():
        return m.String

