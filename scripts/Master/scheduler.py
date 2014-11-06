import rospy
from pygraph.classes.digraph import digraph
from pygraph.readwrite.dot import write
from ros_vision.msg import Filter, FilterList

class Scheduler:
    filters = {}
    def __init__(self):
        pass

    def update_graph(self):
        graph = digraph()

        for f in self.filters.keys():
            graph.add_node(f)

        for f, inputs in self.filters.items():
            for i in inputs:
                edge = ("/".join(i.split("/")[:3]), f)
                print edge
                if not graph.has_edge(edge):
                    graph.add_edge(edge)

        dot = write(graph)
        import gv
        gvv = gv.readstring(dot)
        gv.layout(gvv,'dot')
        gv.render(gvv,'png','/home/mathieu/test.png')


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
            rospy.sleep(1)