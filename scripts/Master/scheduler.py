import rospy
from pygraph.classes.digraph import digraph
from pygraph.readwrite.dot import write
from ros_vision.msg import Filter, FilterList
from pygraph.algorithms.searching import depth_first_search
class Scheduler:
    filters = {}
    graph = None
    i = 0
    def __init__(self):
        pass

    def update_graph(self):
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

        dot = write(self.graph)
        import gv
        gvv = gv.readstring(dot)

        gv.layout(gvv,'dot')
        self.i += 1
        gv.render(gvv,'png','/home/mathieu/test' + str(self.i) + '.png')


    def filter_update_callback(self, msg):
        fc_node_name = msg._connection_header['callerid']
        for f in msg.filters:
            print f
            name = fc_node_name + "/" + f.name
            self.filters[name] = [i.topic for i in f.inputs]
            self.update_graph()

    def add_filter_chain_node(self, name):
        rospy.Subscriber("/%s/filters" % name, FilterList, self.filter_update_callback)

    def run(self):
        while not rospy.is_shutdown():
            if self.graph is not None:
                print "aaa"
                print depth_first_search(self.graph)
            rospy.sleep(1)