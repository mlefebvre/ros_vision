import rospy

class IOManager:
    _instance = None
    _publishers = {}
    _subscribers = {}
    _subscribers_types = {}
    _last_values = {}

    def __init__(self):
        pass

    def _format_topic_name(self, name):
        if not name.startswith("/"):
            if name.startswith("~"):
                name = rospy.get_name() + name[1:]
            else:
                name = rospy.get_name() + "/" + name

        return name

    def create_topic(self, name, topic_type):
        name = self._format_topic_name(name)
        print "Created topic publisher: %s" % name
        self._publishers[name] = rospy.Publisher(name, topic_type.get_ros_type(), queue_size=10)

        #Deleting subscribers on this topic for better performance
        conflict = {}
        for n, subscriber in self._subscribers.items():
            if name == n:
                conflict[name] = subscriber

        for n, subscriber in conflict.items():
            subscriber.unregister()
            del self._subscribers[name]

    def _topic_callback(self, msg):
        topic = msg._connection_header['topic']
        self._last_values[topic] = msg

    def watch_topic(self, name, topic_type):
        name = self._format_topic_name(name)
        if not name in self._publishers:
            self._subscribers[name] = rospy.Subscriber(name, topic_type.get_ros_type(), self._topic_callback)
        self._subscribers_types[name] = topic_type

    def update_value(self, name, value):
        name = self._format_topic_name(name)
        #if
        self._last_values[name] = value

    def get_value(self, name, time=0):
        name = self._format_topic_name(name)
        if name in self._last_values:
            val = self._last_values[name]
            if type(val) != self._subscribers_types[name]:
                return self._subscribers_types[name].from_ros_msg(val)
            else:
                return val
        else:
            return None

    # Singleton
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(IOManager, cls).__new__(cls, *args, **kwargs)
        return cls._instance