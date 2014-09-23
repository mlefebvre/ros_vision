import rospy


class IOManager:
    _instance = None
    _publishers = {}


    def __init__(self):
        pass

    def create_topic(self, name, topic_type):
        self._publishers[name] = rospy.Publisher(name, topic_type.get_ros_type())

    def watch_topic(self, name, topic_type):
        pass

    def update_value(self, name, value):
        pass

    def get_value(self, name, time=0):
        pass

    # Singleton
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(IOManager, cls).__new__(cls, *args, **kwargs)
        return cls._instance