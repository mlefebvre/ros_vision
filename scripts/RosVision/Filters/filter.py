import abc


class Filter:
    __metaclass__ = abc.ABCMeta

    name = ""
    _params = {}

    def __init__(self, name, params=None):
        self.name = name
        if params is not None:
            self._params = params
        self.initialize()

    @abc.abstractmethod
    def initialize(self):
        return

    @abc.abstractmethod
    def execute(self, time=0):
        return

    def set_param(self, name, value):
        print "Param: ", name, value
        self._params[name] = value

    def get_param(self, name):
        return self._params[name]

    def get_params(self):
        return self._params

    def set_output(self, name, value):
        pass

    def get_input(self, name):
        pass

