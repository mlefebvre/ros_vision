#!/usr/bin/env python

import roslib
roslib.load_manifest('ros_vision')
import rospy
from ros_vision.srv import AddInput, AddOutput
from Scheduler.topic_watcher import TopicWatcher


rospy.init_node('scheduler_node')

inputs = {}
outputs = {}
rate = rospy.Rate(30)

def add_input(req):
    if not req.name in inputs:
        inputs[req.name] = tw = TopicWatcher(req.name, req.type)
        print inputs

def add_output(req):
    if not req.name in outputs:
        outputs[req.name] = tw = TopicWatcher(req.name, req.type)
        print outputs

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
        if len(inputs) == len([i for i in inputs if i.get_last_time() != -1]):
            inputs_sorted = sorted(inputs, key=lambda x: x.get_rate())

            first_time = inputs_sorted[0].get_last_time()




    if output_rate != last_rate:
        last_rate = output_rate
        print "New rate:", last_rate
        rate = rospy.Rate(last_rate)

        rate.sleep()



