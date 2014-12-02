#!/usr/bin/env python

import roslib
roslib.load_manifest('ros_vision')
import roslib.message
import rospy
from ros_vision.srv import AddInput, AddOutput
from Scheduler.topic_watcher import TopicWatcher


rospy.init_node('scheduler_node')

inputs = {}
outputs = {}
publishers = {}
rate = rospy.Rate(30)

def add_input(req):
    if not req.name in inputs:
        inputs[req.name] = TopicWatcher(req.name, req.type)
        publishers[req.name] = rospy.Publisher(req.name, roslib.message.get_message_class(req.type))

def add_output(req):
    if not req.name in outputs:
        outputs[req.name] = TopicWatcher(req.name, req.type)

def get_min_rate(l):
    if len(l) > 0:
        rates = [r for r in [d.get_rate() for d in l] if r != 0]
        if len(rates) > 0:
            return min(rates)
    return 0


rospy.Service('~add_input', AddInput, add_input)
rospy.Service('~add_output', AddOutput, add_output)

last_rate = 30
rate = rospy.Rate(last_rate)

while not rospy.is_shutdown():
    output_rate = int(get_min_rate(outputs.values()))
    if output_rate == 0:
        output_rate = 1

    if len(inputs) > 0:
        if len(inputs) == len([i for i in inputs if i.has_messages()]):
            inputs_sorted = sorted(inputs, key=lambda x: x.get_rate())
            first_time = inputs_sorted[0].get_last_time()
            for name, tw in inputs.items():
                msg = tw.get_nearest_message(first_time)






    if output_rate != last_rate:
        last_rate = output_rate
        print "New rate:", last_rate
        rate = rospy.Rate(last_rate)

        rate.sleep()



