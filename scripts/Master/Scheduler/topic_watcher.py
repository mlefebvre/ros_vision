import rospy


class TopicWatcher:
    def __init__(self, topic_name, topic_type, on_message=None):
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.on_message = on_message
        self.last_time = -1
        self.new_message = False
        self.listener = rospy.Subscriber(self.topic_name, self.topic_type, self._topic_callback)
        self.rates = []
        self.last_messages = []
        self.last_times = []

    def _topic_callback(self, msg):
        if hasattr(msg, "header"):
            self.last_time = msg.header.stamp.to_sec()
        else:
            self.last_time = rospy.get_time()

        self.rates.append(rospy.get_time())
        # Delete old rates
        while len(self.rates) > 1:
            if rospy.get_time() - self.rates[0] > 2:
                del self.rates[0]
            else:
                break

        while len(self.rates) > 60:
            del self.rates[0]

        self.last_messages.append(msg)
        self.last_times.append(self.last_time)
        while len(self.last_messages) > 10:
            del self.last_messages[0]
            del self.last_times[0]

        self.new_message = True

        if self.on_message is not None:
            self.on_message(self)

    def has_new_message(self):
        return self.new_message

    def reset_new_message(self):
        self.new_message = False

    def get_topic_name(self):
        return self.topic_name

    def get_last_time(self):
        return self.last_time

    def has_messages(self):
        return len(self.last_messages) > 0

    def get_nearest_message(self, time):
        t = min(self.last_times, key=lambda x: abs(x-time))
        return self.last_messages[self.last_times.index(t)]

    def reset_rate(self):
        self.rates = []

    def get_rate(self):
        if len(self.rates) > 1:
            return len(self.rates) / (self.rates[-1] - self.rates[0])
        else:
            return 0

    def stop(self):
        if self.listener is not None:
            self.listener.unregister()
            self.listener = None
