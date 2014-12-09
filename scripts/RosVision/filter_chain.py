from Filters.filter_factory import FilterFactory
import time
from threading import Lock


class FilterChain:

    _filters = {}
    _filter_list = []
    _filter_stats = {}
    _filter_list_lock = Lock()

    def __init__(self):
        pass

    def set_param(self, name, value):
        n = name.split("/")
        if n[0] in self._filters:
            self._filters[n[0]].set_param(n[1], value)
        else:
            pass

    def get_param(self, name):
        n = name.split("/")
        if n[0] in self._filters:
            return self._filters[n[0]].get_param(n[1])
        else:
            pass

    def create_filter(self, name, filter_type, order=len(_filter_list), params={}):
        self._filter_list_lock.acquire()
        f = FilterFactory.create_filter(name, filter_type, params)
        self._filters[name] = f
        self._filter_list.insert(order, f)
        self._filter_stats[name] = []
        self._filter_list_lock.release()

    def delete_filter(self, name):
        self._filter_list.remove(self._filters[name])
        del self._filters[name]
        del self._filter_stats[name]

    def save(self, file_name):
        pass

    def get_descriptor(self, filter_name):
        pass

    def execute(self):
        self._filter_list_lock.acquire()
        try:
            current_stats = {}
            for f in self._filter_list:
                start = time.time()
                f.execute()
                t = time.time() - start
                current_stats[f.name] = t
                self._filter_stats[f.name].append(t)
                while len(self._filter_stats[f.name]) > 100:
                    del self._filter_stats[f.name][0]
            self._filter_list_lock.release()
        except Exception as e:
            print e.message
            return False
        return current_stats

    def get_average_filter_execution_time(self):
        avg = {}
        for f_name, f_times in self._filter_stats.items():
            if len(f_times) == 0:
                avg[f_name] = 0
            else:
                avg[f_name] = sum(f_times) / len(f_times)
        return avg


    def get_filters(self):
        return self._filter_list


