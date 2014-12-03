#!/usr/bin/env python
import roslib.message
import rospy
from threading import Lock

from topic_watcher import TopicWatcher


# Modifier un jour pour supporter plusieurs chaines de filtres
class Scheduler:
    def __init__(self, workspace):
        self.inputs = {}
        self.outputs = {}
        self.publishers = {}
        self.workspace = workspace
        self.workspace.add_update_listener(self.on_workspace_update)
        self.on_workspace_update()
        self.publisher_lock = Lock()

    def on_workspace_update(self):
        self.update_inputs()
        self.update_outputs()

    def update_inputs(self):
        new_inputs = {}
        for fc in self.workspace.input_topics:
            for topic, topic_type in fc:
                new_inputs[topic] = roslib.message.get_message_class(topic_type)

        # Delete unused topics
        old_inputs = []
        for topic, tw in self.inputs.items():
            if not topic in new_inputs:
                old_inputs.append(topic)
            elif tw.topic_type != new_inputs[topic]:
                old_inputs.append(topic)
        while len(old_inputs) > 0:
            self.outputs[old_inputs[0]].close()
            del self.outputs[old_inputs[0]]
            del old_inputs[0]

        # Subscribe new topics
        for topic, topic_type in new_inputs.items():
            if not topic in self.inputs:
                self.inputs[topic] = TopicWatcher(topic, topic_type, self.on_input_message)
                self.publishers[topic] = rospy.Publisher("/vision_master/scheduler" + topic, topic_type, queue_size=10)

    def update_outputs(self):
        new_outputs = {}
        for fc in self.workspace.output_topics:
            for topic, topic_type in fc:
                new_outputs[topic] = roslib.message.get_message_class(topic_type)

        # Delete unused topics
        old_outputs = []
        for topic, tw in self.outputs.items():
            if not topic in new_outputs:
                old_outputs.append(topic)
            elif tw.topic_type != new_outputs[topic]:
                old_outputs.append(topic)
        while len(old_outputs) > 0:
            self.outputs[old_outputs[0]].close()
            del self.outputs[old_outputs[0]]
            del old_outputs[0]

        # Subscribe new topics
        for topic, topic_type in new_outputs.items():
            if not topic in self.outputs:
                self.outputs[topic] = TopicWatcher(topic, topic_type)

    def get_min_rate(self, l):
        if len(l) > 0:
            rates = [r for r in [d.get_rate() for d in l] if r != 0]
            if len(rates) > 0:
                return min(rates)
        return 0

    def on_input_message(self, src):
        if len(self.inputs) == len([i for i in self.inputs.values() if i.has_messages()]):
            inputs_sorted = sorted(self.inputs.items(), key=lambda x: x[1].get_rate())
            # Slowest message received, publish
            if inputs_sorted[0][1] == src:
                last_time = inputs_sorted[0][1].get_last_time()
                for name, tw in inputs_sorted:
                    self.publishers[name].publish(tw.get_nearest_message(last_time))

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            #output_rate = int(self.get_min_rate(self.outputs.values()))
            #if output_rate == 0:
            #    output_rate = 1

            # if len(self.inputs) > 0:
            #     if len(self.inputs) == len([i for i in self.inputs if i.has_messages()]):
            #         inputs_sorted = sorted(self.inputs, key=lambda x: x.get_rate())
            #         first_time = inputs_sorted[0].get_last_time()
            #         for name, tw in self.inputs.items():
            #             msg = tw.get_nearest_message(first_time)

            #if output_rate != self.last_rate:
            #    self.change_rate(output_rate)

            rate.sleep()



