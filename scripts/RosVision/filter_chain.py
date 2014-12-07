from Filters.filter_factory import FilterFactory


class FilterChain:

    _filters = {}
    _filter_list = []

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
        f = FilterFactory.create_filter(name, filter_type, params)
        self._filters[name] = f
        self._filter_list.insert(order, f)

    def delete_filter(self, name):
        self._filter_list.remove(self._filters[name])
        del self._filters[name]

    def save(self, file_name):
        pass

    def get_descriptor(self, filter_name):
        pass

    def execute(self):
        for f in self._filter_list:
            f.execute()

    def get_filters(self):
        return self._filter_list


