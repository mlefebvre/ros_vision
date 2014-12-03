import abc
import importlib
import os
import pkgutil
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
        if name.find("/") < 0 and self.name is not None:
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
        self._io_manager.update_value(self.get_io_name(name), value)

    def get_input(self, *names):
        topic_names = []
        for name in names:
            topic_names.append(self.get_io_name(name))
        return self._io_manager.get_values(topic_names)

    def get_io_name(self, name):
        if name in self._params:
            name = self._params[name]
        name = self._format_io_name(name)
        return name

    @staticmethod
    def list_descriptors():
        descriptors = {}
        pkgpath = os.path.dirname(os.path.realpath(__file__))

        for _, module, ispkg in pkgutil.iter_modules([pkgpath]):
            if ispkg:
                i = importlib.import_module("RosVision.Filters.%s.filter" % module)
                if hasattr(i, "__dict__"):
                    for n, c in i.__dict__.items():
                        try:
                            if issubclass(c, Filter) and n is not "Filter":
                                descriptors[n] = c.descriptor
                        except:
                            pass

        return descriptors



