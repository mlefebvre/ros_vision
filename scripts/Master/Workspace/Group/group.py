from filter_group_node_wrapper import FilterGroupNodeWrapper
import rospy
from ros_vision.msg import FilterList


class Group:
    def __init__(self, name, filters, on_update):
        self.name = name
        self.wrapper = FilterGroupNodeWrapper(name, filters)
        self.subscriber = rospy.Subscriber("%s/filters" % name, FilterList, self._update_callback)
        self.filters = {}
        self.on_update = on_update
        self.ready = False

    def _update_callback(self, msg):
        filters = {}
        for f in msg.filters:
            name = self.name + "/" + f.name
            filters[name] = f
        self.filters = filters
        self.ready = True
        self.on_update()

    def get_inputs(self):
        inputs = {}
        for name, f in self.filters.items():
            inputs[name] = [(i.topic, i.type) for i in f.inputs]
        return inputs

    def get_outputs(self):
        outputs = {}
        for name, f in self.filters.items():
            outputs[name] = [(o.topic, o.type) for o in f.outputs]
        return outputs

    def get_filter_info(self, filter_name):
        if filter_name in self.filters:
            return self.filters[filter_name]
        return None

    def kill(self):
        self.subscriber.unregister()
        self.wrapper.kill()

    def is_ready(self):
        return self.ready