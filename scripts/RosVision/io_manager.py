import rospy
from ros_vision.msg import StartSignal
#http://wiki.ros.org/message_filters


class Singleton(object):
  _instances = {}
  def __new__(class_, *args, **kwargs):
    if class_ not in class_._instances:
        class_._instances[class_] = super(Singleton, class_).__new__(class_, *args, **kwargs)
    return class_._instances[class_]


class IOManager(Singleton):
    MAX_LIST_SIZE = 10
    _instance = None
    _publishers = {}
    _subscribers = {}
    _subscribers_types = {}
    _last_values = {}
    _signal_time = 0

    def _signal_callback(self, signal):
        times = []
        signal_time = 0
        for i, name in enumerate(signal.input_name):
            times.append(signal.input_time[i].to_sec())
        if len(times) > 0:
            signal_time = max(times)
        self._signal_time = signal_time
        print rospy.get_name(), "Signal ", self._signal_time

    def received_signal_since(self, time):
        return self._signal_time > time or time == 0

    def get_last_signal(self):
        return self._signal_time

    def connect_scheduler(self):
        self.signal_topic = rospy.Subscriber('/vision_master/scheduler/signal', StartSignal, self._signal_callback)

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
            self._last_values[name] = {}

        d = self._last_values[name]
        d[time] = value

        while len(d) > IOManager.MAX_LIST_SIZE:
            del d[min(d.keys())]

    def _topic_callback(self, msg, topic):
        if hasattr(msg, "header"):
            time = msg.header.stamp.to_sec()
            print rospy.get_name(), ": ", "--------------------", topic, repr(time)
        else:
            time = rospy.get_time()

        self._add_value(topic, msg, time)

    def watch_topic(self, name, topic_type):
        name = self.format_topic_name(name)
        if not name in self._publishers:
            self._subscribers[name] = rospy.Subscriber(name, topic_type.get_ros_type(), self._topic_callback, callback_args=name)
        self._subscribers_types[name] = topic_type

    def update_value(self, name, value):
        if hasattr(value, 'get_time'):
            ####################################################
            value.set_time(self._signal_time)

            ####################################################
            time = value.get_time()
        else:
            time = rospy.get_time()

        name = self.format_topic_name(name)
        print rospy.get_name(), ": ", "--------------------", name, repr(time)

        self._add_value(name, value, time)

        pub = self._publishers[name]
        if pub.get_num_connections() > 0:
            pub.publish(value.to_ros_msg())

    def get_value(self, name, wait=True):
        name = self.format_topic_name(name)
        if name in self._last_values:
            d = self._last_values[name]
            if len(d) > 0:
                max_time = max(d.keys())
                t = max_time
                print rospy.get_name(), max_time, self._signal_time, max_time != self._signal_time
                if wait and max_time != self._signal_time:
                    if max_time > self._signal_time:
                        t = min(d.keys(), key=lambda x:abs(x-self._signal_time))
                    else:
                        #max_wait = ((max_time - min(d.keys())) / len(d)) * 2
                        max_wait = 0.05
                        #if name is self._subscribers:
                        #    max_wait = 1.0
                        max_wait_time = rospy.get_time() + max_wait
                        print rospy.get_name(), ": ", rospy.get_time(), max_wait, max_wait_time
                        print rospy.get_name(), ": ", "wait", name, repr(max_time), repr(self._signal_time)
                        while rospy.get_time() < max_wait_time:
                            rospy.sleep(1.0/1000)
                            max_time2 = max(d.keys())
                            if max_time2 > max_time:
                                t = max_time2
                                print "break"
                                break
                val = d[t]
                if type(val) != self._subscribers_types[name]:
                    return self._subscribers_types[name](val)
                else:
                    return val
        return None

    # # Singleton
    # def __new__(cls, *args, **kwargs):
    #     if not cls._instance:
    #         cls._instance = super(IOManager, cls).__new__(cls, *args, **kwargs)
    #     return cls._instance