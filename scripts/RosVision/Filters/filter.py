import abc
from ..io_manager import IOManager

class Filter:
    __metaclass__ = abc.ABCMeta

    name = ""
    _params = None
    descriptor = None
    _io_manager = IOManager()

    def __init__(self, name, params):
        self.name = name
        self._init_params(params)
        self._init_io(params)
        self.initialize()
        print "=============================="

    def _init_params(self, params):
        for p in self.descriptor.get_parameters():
            if not p.get_name() in params:
                params[p.get_name()] = p.get_default_value()
        self._params = params

    def _format_io_name(self, name):
        if name.find("/") < 0:
            name = self.name + "/" + name
        return name

    def _init_io(self, params):
        # Outputs
        for i in self.descriptor.get_outputs():
            name = i.get_name()
            if i.get_name() in params:
                name = params[i.get_name()]
            name = self._format_io_name(name)
            self._io_manager.create_topic(name, i.get_io_type())

        # Inputs
        for i in self.descriptor.get_inputs():
            name = i.get_name()
            if i.get_name() in params:
                name = params[i.get_name()]
            name = self._format_io_name(name)
            self._io_manager.watch_topic(name, i.get_io_type())

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
        if name in self._params:
            name = self._params[name]
        name = self._format_io_name(name)
        self._io_manager.update_value(name, value)

    def get_input(self, name):
        if name in self._params:
            name = self._params[name]
        name = self._format_io_name(name)
        return self._io_manager.get_value(name)

    def get_topic_name(self, name):
        return IOManager().format_topic_name(self._format_io_name(name))

    def get_descriptor(self):
        return self.descriptor



