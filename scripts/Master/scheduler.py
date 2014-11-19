import rospy
from ros_vision.msg import FilterList
import networkx as nx
import roslib


class Scheduler:
    groups = {}
    filter_inputs = {}
    filter_outputs = {}
    graphs = []
    update = False
    start_topics = []
    end_topics = []
    end_listeners = []


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

    def _find_start_topics(self):
        start_topics = []
        for graph in self.graphs:
            s = []
            for n in graph.nodes():
                if len(graph.predecessors(n)) == 0:
                    sucessors = graph.successors(n)
                    if len(sucessors) != 0:
                        s.append(graph.get_edge_data(n, sucessors[0])["topic"])
            start_topics.append(s)
        return start_topics

    def _find_end_topics(self):
        end_topics = []
        for graph in self.graphs:
            e = []
            for n in graph.nodes():
                if len(graph.successors(n)) == 0:
                    e.extend(self.filter_outputs[n])
            end_topics.append(e)
        return end_topics

    def _topic_callback(self, msg, topic_name):
        if hasattr(msg, "header"):
            time = msg.header.stamp.to_sec()
        else:
            time = rospy.get_time()
        print topic_name, time

    def run(self):
        while sum([1 for g in self.groups.values() if g is None]) > 0:
            rospy.sleep(0.5)

        while not rospy.is_shutdown():
            if self.update:
                self.update = False
                self.start_topics = self._find_start_topics()
                self.end_topics = self._find_end_topics()

                for l in self.end_listeners:
                    l.unregister()
                self.end_listeners = []


                for group in self.end_topics:
                    for topic_name, topic_type in group:
                        topic_type = roslib.message.get_message_class(topic_type)
                        l = rospy.Subscriber(topic_name, topic_type, self._topic_callback, callback_args=topic_name)
                        self.end_listeners.append(l)

            rospy.sleep(1)
