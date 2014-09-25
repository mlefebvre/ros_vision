import abc


class IOObject(object):
    value = None

    def __init__(self, value):
        self.value = value
        #self.time = time

    @abc.abstractmethod
    def to_ros_msg(self):
        return

    @staticmethod
    def from_ros_msg(data):
        return

    @staticmethod
    def get_ros_type():
        return None

    def get_value(self):
        return self.value

