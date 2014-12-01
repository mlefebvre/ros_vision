import rospy
import roslib


class TopicWatcher:
    def __init__(self, topic_name, topic_type, on_message=None):
        self.topic_name = topic_name
        self.topic_type = roslib.message.get_message_class(topic_type)
        self.on_message = on_message
        self.last_time = -1
        self.new_message = False
        self.listener = rospy.Subscriber(self.topic_name, self.topic_type, self._topic_callback)

    def _topic_callback(self, msg):
        if hasattr(msg, "header"):
            self.last_time = msg.header.stamp.to_sec()
        else:
            self.last_time = rospy.get_time()

        self.new_message = True

        if self.on_message is not None:
            print repr(rospy.get_time()), "RECV"
            self.on_message(self)

    def has_new_message(self):
        return self.new_message

    def reset_new_message(self):
        self.new_message = False

    def get_topic_name(self):
        return self.topic_name

    def get_last_time(self):
        return self.last_time

    def stop(self):
        if self.listener is not None:
            self.listener.unregister()
            self.listener = None
