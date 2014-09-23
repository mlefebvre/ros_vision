from Resize.resize_filter import ResizeFilter


class FilterFactory:
    def __init__(self):
        pass

    @staticmethod
    def create_filter(name, filter_type, params=None):
        f = ResizeFilter(name, params)
        return f