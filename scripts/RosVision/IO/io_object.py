import abc


class IOObject(object):
    __metaclass__ = abc.ABCMeta

    value = None

    def __init__(self, value):
        self.value = value

    def to_ros_msg(self):
        if type(self.value) == self.get_ros_type():
            return self.value
        else:
            return self._to_ros_msg()

    @abc.abstractmethod
    def _from_ros_msg(self, value):
        return

    @abc.abstractmethod
    def _to_ros_msg(self):
        return

    @staticmethod
    def get_ros_type():
        return None

    def get_value(self):
        if type(self.value) == self.get_ros_type():
            self.value = self._from_ros_msg(self.value)
        return self.value

