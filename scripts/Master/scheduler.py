import rospy
from pygraph.classes.digraph import digraph
from pygraph.readwrite.dot import write
from ros_vision.msg import Filter, FilterList
from pygraph.algorithms.searching import depth_first_search
class Scheduler:
    filters = {}
    graph = None
    update = False

    i = 0
    def __init__(self):
        pass

    def update_graph(self):
        old_graph = self.graph
        self.graph = digraph()

        for f in self.filters.keys():
            self.graph.add_node(f)

        for f, inputs in self.filters.items():
            for i in inputs:
                input_filter = "/".join(i.split("/")[:3])
                if not self.graph.has_node(input_filter):
                    self.graph.add_node(input_filter)
                edge = (input_filter, f)
                if not self.graph.has_edge(edge):
                    self.graph.add_edge(edge)

        if old_graph is not None and write(self.graph) != write(old_graph):
            self.update = True

        # dot = write(self.graph)
        # import gv
        # gvv = gv.readstring(dot)
        #
        # gv.layout(gvv,'dot')
        # self.i += 1
        # gv.render(gvv,'png','/home/mathieu/test' + str(self.i) + '.png')


    def filter_update_callback(self, msg):
        fc_node_name = msg._connection_header['callerid']
        for f in msg.filters:
            #print f
            name = fc_node_name + "/" + f.name
            self.filters[name] = [i.topic for i in f.inputs]
            self.update_graph()

    def add_filter_chain_node(self, name):
        rospy.Subscriber("/%s/filters" % name, FilterList, self.filter_update_callback)

    def _find_root_nodes(self):
        root_nodes = []
        nodes = self.graph.nodes()
        edges = self.graph.edges()
        for n in nodes:
            inputs = sum([1 for e in edges if e[1] == n])
            if inputs == 0:
                root_nodes.append(n)
        return root_nodes

    def _find_end_nodes(self):
        end_nodes = []
        nodes = self.graph.nodes()
        edges = self.graph.edges()
        for n in nodes:
            inputs = sum([1 for e in edges if e[0] == n])
            if inputs == 0:
                end_nodes.append(n)
        return end_nodes

    def run(self):
        while not rospy.is_shutdown():
            if self.graph is not None:
                if self.update:
                    self.update = False
                    print self._find_root_nodes()
                    print self._find_end_nodes()

            #Trouver graphes déconnectés

            rospy.sleep(1)