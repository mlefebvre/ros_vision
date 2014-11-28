import rospy
from ros_vision.msg import FilterList, StartSignal
import networkx as nx
from group_watcher import GroupWatcher
from threading import Lock


class Scheduler:
    def __init__(self):
        self.groups = {}
        self.filter_inputs = {}
        self.filter_outputs = {}
        self.graphs = []
        self.update = False
        self.input_topics = []
        self.output_topics = []
        self.group_watchers = []
        self.graph_lock = Lock()
        self.signal_topic = rospy.Publisher('/vision_master/scheduler/signal', StartSignal, queue_size=10, latch=True)

    def update_graph(self):
        self.graph_lock.acquire()

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
        self.graph_lock.release()

        # import matplotlib.pyplot as plt
        # nx.draw_graphviz(graph)
        # plt.show()

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

    def _on_loop_complete(self, source):
        signal = StartSignal()
        times = []
        for i in source.input_topic_watchers:
            signal.group_names.append("/%s" % i.get_topic_name().split("/")[1])
            times.append(rospy.Time.from_sec(i.get_last_time()))

        signal.input_time = max(times)
        self.signal_topic.publish(signal)

    def run(self):
        while sum([1 for g in self.groups.values() if g is None]) > 0:
            rospy.sleep(0.5)

        while not rospy.is_shutdown():
            if self.update:
                self.update = False
                self.graph_lock.acquire()
                self.input_topics = self._find_input_topics()
                self.output_topics = self._find_output_topics()
                self.graph_lock.release()

                while len(self.group_watchers) > 0:
                    self.group_watchers[0].stop()
                    del self.group_watchers[0]

                self.group_watchers = []
                for i in range(len(self.input_topics)):
                    self.group_watchers.append(GroupWatcher(self.input_topics[i], self.output_topics[i], self._on_loop_complete))

            rospy.sleep(1)
