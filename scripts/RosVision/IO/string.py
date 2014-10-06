from io_object import IOObject
import std_msgs.msg as m


class String(IOObject):
    def _to_ros_msg(self):
        return m.String(self.value)

    @staticmethod
    def _from_ros_msg(self, data):
        return data

    @staticmethod
    def get_ros_type():
        return m.String

