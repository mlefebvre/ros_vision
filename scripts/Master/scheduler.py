import rospy
from ros_vision.msg import FilterList, StartSignal
import networkx as nx
import roslib


class Scheduler:
    groups = {}
    filter_inputs = {}
    filter_outputs = {}
    graphs = []
    update = False
    input_topics = []
    output_topics = []
    input_listeners = []
    output_listeners = []
    last_input_time = []
    output_status = []
    loop_count = []

    i = 0
    def __init__(self):
        pass

    def update_graph(self):
        graph = nx.DiGraph()

        graph.add_nodes_from(self.filter_inputs.keys())

        for f, inputs in self.filter_inputs.items():
            for i, t in inputs:
                input_filter = "/".join(i.split("/")[:3])
                if not graph.has_node(input_filter):
                    graph.add_node(input_filter)

                if f != input_filter:
                    if not graph.has_edge(input_filter, f):
                        graph.add_edge(input_filter, f, {'topic': (i, t)})

        graphs = []

        edges = graph.edges()
        for subgraph in nx.connected_component_subgraphs(graph.to_undirected()):
            g = nx.DiGraph()
            g.add_nodes_from(subgraph)
            for e in edges:
                if g.has_node(e[0]) or g.has_node(e[1]):
                    g.add_edge(e[0], e[1], attr_dict=graph.get_edge_data(e[0], e[1]))
            graphs.append(g)

        self.graphs = graphs
        self.update = True

    def filter_update_callback(self, msg, fc_node_name):
        self.groups[fc_node_name] = msg
        for f in msg.filters:
            name = fc_node_name + "/" + f.name
            self.filter_inputs[name] = [(i.topic, i.type) for i in f.inputs]
            self.filter_outputs[name] = [(o.topic, o.type) for o in f.outputs]

        self.update_graph()

    def add_filter_chain_group(self, name):
        name = "/" + name
        self.groups[name] = None
        rospy.Subscriber("%s/filters" % name, FilterList, self.filter_update_callback, callback_args=name)

    def _find_input_topics(self):
        input_topics = []
        for graph in self.graphs:
            s = []
            for n in graph.nodes():
                if len(graph.predecessors(n)) == 0:
                    sucessors = graph.successors(n)
                    if len(sucessors) != 0:
                        s.append(graph.get_edge_data(n, sucessors[0])["topic"])
            input_topics.append(s)
        return input_topics

    def _find_output_topics(self):
        output_topics = []
        for graph in self.graphs:
            e = []
            for n in graph.nodes():
                if len(graph.successors(n)) == 0:
                    e.extend(self.filter_outputs[n])
            output_topics.append(e)
        return output_topics

    def _input_topic_callback(self, msg, topic_name):
        if hasattr(msg, "header"):
            time = msg.header.stamp.to_sec()
        else:
            time = rospy.get_time()
        for group_id, group in enumerate(self.last_input_time):
            if topic_name in group:
                group[topic_name] = time
                if self.loop_count[group_id] == 0:
                    if sum([1 for t in group if t == -1]) == 0:
                        print "Signal"


    def _output_topic_callback(self, msg, topic_name):
        for group_id, group in enumerate(self.output_status):
            if topic_name in group:
                #Lock ici?
                group[topic_name] = True
                if len(group.keys()) == sum([1 for t in group.values() if t]):
                    print self.last_input_time[group_id]
                    self.loop_count[group_id] += 1
                    for t in group.keys():
                        group[t] = False

    def run(self):
        while sum([1 for g in self.groups.values() if g is None]) > 0:
            rospy.sleep(0.5)

        while not rospy.is_shutdown():
            if self.update:
                self.update = False
                self.input_topics = self._find_input_topics()
                self.output_topics = self._find_output_topics()

                while len(self.input_listeners) > 0:
                    self.input_listeners[0].unregister()
                    del self.input_listeners[0]

                while len(self.output_listeners) > 0:
                    self.output_listeners[0].unregister()
                    del self.output_listeners[0]

                self.last_input_time = []
                self.loop_count = []
                for group in self.input_topics:
                    self.last_input_time.append({})
                    self.loop_count.append(0)
                    for topic_name, topic_type in group:
                        self.last_input_time[-1][topic_name] = -1
                        topic_type = roslib.message.get_message_class(topic_type)
                        l = rospy.Subscriber(topic_name, topic_type, self._input_topic_callback, callback_args=topic_name)
                        self.input_listeners.append(l)

                self.output_status = []
                for group in self.output_topics:
                    self.output_status.append({})
                    for topic_name, topic_type in group:
                        self.output_status[-1][topic_name] = False
                        topic_type = roslib.message.get_message_class(topic_type)
                        l = rospy.Subscriber(topic_name, topic_type, self._output_topic_callback, callback_args=topic_name)
                        self.output_listeners.append(l)

            rospy.sleep(1)
