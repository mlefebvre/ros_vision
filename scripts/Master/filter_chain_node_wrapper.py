from node import Node
import rosparam
import rospy
from ros_vision.srv import CreateFilter


class FilterChainNodeWrapper:
    def __init__(self, name, filters={}):
        self.name = name
        self.node = Node("ros_vision", "filter_chain_node.py", name)
        self.reset_params()
        self.node.run()
        create_filter_srv_name = '/%s/create_filter' % name
        rospy.wait_for_service(create_filter_srv_name)
        self.create_filter = rospy.ServiceProxy(create_filter_srv_name, CreateFilter)

        i = 0
        for filter_name, parameters in filters:
            i += 1
            filter_type = parameters['type']
            del parameters['type']

            for parameter_name, parameter_value in parameters:
                rosparam.set_param('/%s/%s/%s' % (name, filter_name, parameter_name), str(parameter_value))

            self.create_filter(filter_name, filter_type, i)
            # TODO: T'es rendu ici

    def reset_params(self):
        for p in rosparam.list_params(self.name):
            rosparam.delete_param(p)

    def kill(self):
        self.node.kill()