import rospy
import ros_vision.msg
import os
import pkgutil
import importlib
from threading import Lock


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
    _last_values_locks = {}
    _signal_time = 0
    _gc_thread = None


    def run(self):
        self.signal_topic = rospy.Subscriber('/vision_master/scheduler/signal', ros_vision.msg.StartSignal, self._signal_callback)
        rospy.Timer(rospy.Duration(0.05), self._garbage_collector)


    def _garbage_collector(self, event):
        for name, values in self._last_values.items():
            self._last_values_locks[name].acquire()
            while len(values) > IOManager.MAX_LIST_SIZE:
                min_time = min(values.keys())
                del values[min_time]
            self._last_values_locks[name].release()


    def _signal_callback(self, signal):
        if rospy.get_name() in signal.group_names:
            self._signal_time = signal.input_time

    def received_signal_since(self, time):
        return self._signal_time > time or time == 0

    def get_last_signal(self):
        return self._signal_time

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
        if not name in self._publishers:
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

    def get_value(self, name, wait=True):
        name = self.format_topic_name(name)
        if name in self._last_values:
            self._last_values_locks[name].acquire()
            d = self._last_values[name]
            if len(d) > 0:
                max_time = max(d.keys())
                t = max_time
                if wait and max_time != self._signal_time:
                    if max_time > self._signal_time:
                        t = min(d.keys(), key=lambda x:abs(x-self._signal_time))
                    else:
                        max_wait = 0.05
                        max_wait_time = rospy.get_time() + max_wait
                        while rospy.get_time() < max_wait_time:
                            print "wait"
                            rospy.sleep(1.0/1000)
                            max_time2 = max(d.keys())
                            if max_time2 > max_time:
                                t = max_time2
                                break
                val = d[t]
                if type(val) != self._subscribers_types[name]:
                    val = self._subscribers_types[name](val)
                    d[t] = val
                self._last_values_locks[name].release()
                return val
        return None

        @staticmethod
        def create_filter_message_from_descriptor(d):
            filter = ros_vision.msg.Filter()
            filter.name = d.get_name()
            filter.type = d.get_name()
            filter.description = d.description

            for i in d.get_inputs():
                d = ros_vision.msg.IODescriptor()
                name = i.get_name()
                d.name = name
                d.type = str(i.get_io_type().get_ros_type()._type)
                d.topic = IOManager().format_topic_name(filter.name + "/" + name)
                filter.inputs.append(d)

            for o in d.get_outputs():
                d = ros_vision.msg.IODescriptor()
                name = o.get_name()
                d.name = name
                d.type = str(o.get_io_type().get_ros_type()._type)
                d.topic = IOManager().format_topic_name(filter.name + "/" + name)
                filter.outputs.append(d)

            for p in d.get_parameters():
                parameter = ros_vision.msg.Parameter()
                parameter.name = p.get_name()
                parameter.description = p.get_description()
                parameter.type = p.get_type().__name__
                parameter.default = str(p.get_default_value())
                parameter.min = str(p.get_min_value())
                parameter.max = str(p.get_max_value())
                filter.parameters.append(parameter)

            return filter

        @staticmethod
        def create_filter_message_from_filter(f):
            filter = IOManager.create_filter_message_from_descriptor(f.descriptor)
            filter.name = f.name

            for idx, i in enumerate(f.descriptor.get_inputs()):
                filter.inputs[idx].topic = IOManager().format_topic_name(f.get_io_name(filter.inputs[idx].name))

            for idx, o in enumerate(f.descriptor.get_outputs()):
                filter.outputs[idx].topic = IOManager().format_topic_name(f.get_io_name(filter.outputs[idx].name))

            return filter

    # # Singleton
    # def __new__(cls, *args, **kwargs):
    #     if not cls._instance:
    #         cls._instance = super(IOManager, cls).__new__(cls, *args, **kwargs)
    #     return cls._instance