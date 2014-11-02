from node import Node
import rosparam
import rospy
from ros_vision.srv import CreateFilter


class FilterChainNodeWrapper:
    def __init__(self, name, filters):
        self.name = name
        self.node = Node("ros_vision", "filter_chain_node.py", name)
        self.reset_params()
        self.node.run()
        create_filter_srv_name = '/%s/create_filter' % name
        rospy.wait_for_service(create_filter_srv_name)
        self.create_filter = rospy.ServiceProxy(create_filter_srv_name, CreateFilter)

        i = 0
        for filter_name, params in sorted(filters.items(), key=lambda x: x[1]['order']):
            i += 1
            filter_type = params['type']
            del params['order']
            del params['type']
            for name, value in params.items():
                rosparam.set_param('/%s/%s/%s' % (self.name, filter_name, name), str(value))

            self.create_filter(filter_name, filter_type, i)

    def reset_params(self):
        for p in rosparam.list_params(self.name):
            rosparam.delete_param(p)

    def kill(self):
        self.node.kill()