from collections import OrderedDict
import networkx as nx
from threading import Lock
import matplotlib.pyplot as plt
from Group.group import Group


class Workspace:
    def __init__(self):
        self.name = None
        self.graph_lock = Lock()
        self.groups = OrderedDict()
        self.filter_chains = []
        self.input_topics = []
        self.output_topics = []
        self.update_listeners = []

    def update_workspace(self):
        self.graph_lock.acquire()

        graph = nx.DiGraph()
        filter_inputs = {}
        for g in self.groups.values():
            inputs = g.get_inputs()
            for k, v in inputs.items():
                filter_inputs[k] = v

        graph.add_nodes_from(filter_inputs.keys())

        for f, inputs in filter_inputs.items():
            for i, t in inputs:
                input_filter = "/".join(i.split("/")[:3])
                if not graph.has_node(input_filter):
                    graph.add_node(input_filter)

                if f != input_filter:
                    if not graph.has_edge(input_filter, f):
                        graph.add_edge(input_filter, f, {'topic': (i, t)})

        filter_chains = []

        edges = graph.edges()
        for subgraph in nx.connected_component_subgraphs(graph.to_undirected()):
            g = nx.DiGraph()
            for n in subgraph.nodes():
                info = None
                group_name = n[:n.rfind("/")]
                if group_name in self.groups:
                    info = self.groups[group_name].get_filter_info(n)
                    g.add_node(n, info=info)
            for e in edges:
                if g.has_node(e[0]) or g.has_node(e[1]):
                    g.add_edge(e[0], e[1], attr_dict=graph.get_edge_data(e[0], e[1]))
            filter_chains.append(g)

        self.filter_chains = filter_chains
        self.input_topics = self._find_input_topics()
        self.output_topics = self._find_output_topics()

        self.graph_lock.release()
        for listener in self.update_listeners:
            listener()

    def _on_group_update(self):
        self.update_workspace()

    def add_group(self, name, filters=OrderedDict()):
        name = "/" + name
        self.groups[name] = Group(name, filters, self._on_group_update)

    def _find_input_topics(self):
        input_topics = []
        for graph in self.filter_chains:
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
        filter_outputs = {}
        for g in self.groups.values():
            outputs = g.get_outputs()
            for k, v in outputs.items():
                filter_outputs[k] = v

        for graph in self.filter_chains:
            e = []
            for n in graph.nodes():
                if len(graph.successors(n)) == 0:
                    e.extend(filter_outputs[n])
            output_topics.append(e)
        return output_topics

    def draw(self):
        nx.draw_graphviz(self.filter_chains[0])
        plt.show()

    def reset(self):
        for g in self.groups.values():
            g.kill()
        self.groups = OrderedDict()
        self.filter_chains = []
        self.input_topics = []
        self.output_topics = []

    def get_filter_groups_names(self):
        return self.groups.keys()

    def is_ready(self):
        return len([g for g in self.groups.values() if g.is_ready()]) == len(self.groups)

    def add_update_listener(self, listener):
        self.update_listeners.append(listener)