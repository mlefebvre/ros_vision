class ParameterDescriptor:
    name = ""
    description = ""
    param_type = None
    default_value = 0
    min_value = None
    max_value = None

    def __init__(self, name, description, param_type, default_value, min_value=None, max_value=None):
        self.name = name
        self.description = description
        self.param_type = param_type
        self.default_value = default_value
        self.min_value = min_value
        self.max_value = max_value

    def get_name(self):
        return self.name

    def get_description(self):
        return self.description

    def get_type(self):
        return self.param_type

    def get_default_value(self):
        return self.default_value

    def get_min_value(self):
        return self.min_value

    def get_max_value(self):
        return self.max_value