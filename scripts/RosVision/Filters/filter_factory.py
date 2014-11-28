import importlib
import os
import pkgutil
from filter import Filter


class FilterFactory:
    def __init__(self):
        pass

    @staticmethod
    def create_filter(name, filter_type, params={}):
        pkgpath = os.path.dirname(os.path.realpath(__file__))
        for _, module, _ in pkgutil.iter_modules([pkgpath]):
            if module.lower() == filter_type.lower():
                i = importlib.import_module("RosVision.Filters.%s.filter" % module)
                if hasattr(i, "__dict__"):
                    for n, c in i.__dict__.items():
                        try:
                            if issubclass(c, Filter):
                                return c(name, params)
                        except:
                            pass

    @staticmethod
    def list_descriptors():
        descriptors_list = []
        pkgpath = os.path.dirname(os.path.realpath(__file__))
        for _, module, ispkg in pkgutil.iter_modules([pkgpath]):
            if ispkg:
                i = importlib.import_module("RosVision.Filters.%s.filter" % module)
                if hasattr(i, "__dict__"):
                    for n, c in i.__dict__.items():
                        try:
                            if issubclass(c, Filter) and n is not "Filter":
                                descriptors_list.append(c.descriptor)
                        except:
                            pass

        return descriptors_list
