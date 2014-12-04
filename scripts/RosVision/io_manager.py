import rospy
from ros_vision.msg import Workspace
from threading import Lock


class Singleton(object):
  _instances = {}
  def __new__(class_, *args, **kwargs):
    if class_ not in class_._instances:
        class_._instances[class_] = super(Singleton, class_).__new__(class_, *args, **kwargs)
    return class_._instances[class_]


class IOManager(Singleton):
    SCHEDULER_NAME = '/vision_master/scheduler'
    MAX_LIST_SIZE = 10
    _instance = None
    _publishers = {}
    _subscribers = {}
    _subscribers_types = {}
    _last_values = {}
    _last_values_locks = {}
    _signal_time = 0
    _gc_thread = None

    def run(self):
        rospy.Timer(rospy.Duration(0.05), self._garbage_collector)
        #rospy.Subscriber("/vision_master/workspace", Workspace, self._on_workspace_update)

    def _on_workspace_update(self, msg):
        names = ["/" + fg.name for fg in msg.filter_groups]
        for topic in self._subscribers.keys():
            ok = False
            for n in names:
                if topic.startswith(n):
                    ok = True
                    break
            if not ok:
                s = self._subscribers[topic]
                real_topic = s.name
                if not real_topic.startswith(self.SCHEDULER_NAME):
                    s.unregister()
                    real_topic = self.SCHEDULER_NAME + topic
                    topic_type = self._subscribers_types[topic].get_ros_type()
                    self._subscribers[topic] = rospy.Subscriber(real_topic, topic_type, self._topic_callback, callback_args=topic)

    def _garbage_collector(self, event):
        for name, values in self._last_values.items():
            self._last_values_locks[name].acquire()
            while len(values) > IOManager.MAX_LIST_SIZE:
                min_time = min(values.keys())
                del values[min_time]
            self._last_values_locks[name].release()

    def format_topic_name(self, name):
        if not name.startswith("/"):
            if name.startswith("~"):
                name = rospy.get_name() + name[1:]
            else:
                name = rospy.get_name() + "/" + name

        return name

    def create_topic(self, name, topic_type):
        name = self.format_topic_name(name)
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

    def _add_value(self, name, value, time):
        if not name in self._last_values:
            self._last_values_locks[name] = Lock()
            self._last_values[name] = {}
        self._last_values[name][time] = value

    def _topic_callback(self, msg, topic):
        if hasattr(msg, "header"):
            time = msg.header.stamp.to_sec()
        else:
            time = rospy.get_time()

        self._add_value(topic, msg, time)

    def watch_topic(self, name, topic_type):
        name = self.format_topic_name(name)
        if not name in self._publishers and not name in self._subscribers:
            self._subscribers[name] = rospy.Subscriber(name, topic_type.get_ros_type(), self._topic_callback, callback_args=name)
        self._subscribers_types[name] = topic_type

    def update_value(self, name, value):
        if hasattr(value, 'get_time'):
            time = value.get_time()
        else:
            time = rospy.get_time()

        name = self.format_topic_name(name)

        self._add_value(name, value, time)

        pub = self._publishers[name]
        if pub.get_num_connections() > 0:
            pub.publish(value.to_ros_msg())

    def _get_value(self, name, time=0):
        val = None
        if name in self._last_values:
            self._last_values_locks[name].acquire()
            d = self._last_values[name]
            if len(d) > 0:
                if time == 0:
                    t = max(t for t in d.keys())
                else:
                    t = min(d.keys(), key=lambda x: abs(x-time))
                val = d[t]
                if type(val) != self._subscribers_types[name]:
                     val = self._subscribers_types[name](val)
                     d[t] = val

            self._last_values_locks[name].release()

        return val

    def _is_extern(self, name):
        if name in self._subscribers:
            return self._subscribers[name].name.startswith(self.SCHEDULER_NAME)
        return False

    def _get_last_time(self, name):
        if name in self._last_values:
            d = self._last_values[name]
            if len(d) > 0:
                return max(t for t in d.keys())
        return 0

    def get_values(self, names):
        values = []
        t_names = [self.format_topic_name(name) for name in names]

        time = 0
        for name in t_names:
            if not self._is_extern(name):
                t = self._get_last_time(name)
                if t > time:
                    time = t

        for name in t_names:
            values.append(self._get_value(name, time))

        if len(values) == 1:
            return values[0]
        else:
            return tuple(values)
