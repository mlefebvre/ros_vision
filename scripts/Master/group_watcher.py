import rospy
from topic_watcher import TopicWatcher


class GroupWatcher(object):
    def __init__(self, input_topics, output_topics, on_loop_complete=None):
        self.input_topic_watchers = []
        self.output_topic_watchers = []
        self.on_loop_complete = on_loop_complete
        self.loop_count = -1

        for topic_name, topic_type in input_topics:
            tw = TopicWatcher(topic_name, topic_type, self._on_input_message)
            self.input_topic_watchers.append(tw)

        for topic_name, topic_type in output_topics:
            tw = TopicWatcher(topic_name, topic_type, self._on_output_message)
            self.output_topic_watchers.append(tw)

    def _on_input_message(self, source):
        if self.loop_count == -1:
            if sum([1 for t in self.input_topic_watchers if t.get_last_time() == -1]) == 0:
                if self.on_loop_complete is not None:
                    self.loop_count = 0
                    self.on_loop_complete(self)

    from threading import  Lock
    lock = Lock()

    def _on_output_message(self, source):
        self.lock.acquire()
        if len(self.output_topic_watchers) == sum([1 for t in self.output_topic_watchers if t.has_new_message()]):
            for t in self.output_topic_watchers:
                t.reset_new_message()
            self.loop_count += 1
            if self.on_loop_complete is not None:
                while len(self.input_topic_watchers) != sum([1 for t in self.input_topic_watchers if t.has_new_message()]):
                    rospy.sleep(1.0/1000)
                for t in self.input_topic_watchers:
                    t.reset_new_message()
                self.on_loop_complete(self)
        self.lock.release()

    def stop(self):
        while len(self.input_topic_watchers) > 0:
            self.input_topic_watchers[0].stop()
            del self.input_topic_watchers[0]
        while len(self.output_topic_watchers) > 0:
            self.output_topic_watchers[0].stop()
            del self.output_topic_watchers[0]





