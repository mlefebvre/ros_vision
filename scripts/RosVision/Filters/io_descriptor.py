class IODescriptor:
    name = ""
    description = ""
    io_type = None

    def __init__(self, name, description, io_type):
        self.name = name
        self.description = description
        self.io_type = io_type

    def get_name(self):
        return self.name

    def get_description(self):
        return self.description

    def get_io_type(self):
        return self.io_type