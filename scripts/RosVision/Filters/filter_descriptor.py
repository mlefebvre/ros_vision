class FilterDescriptor:
    name = ""
    description = ""
    parameters = []
    inputs = []
    outputs = []

    def __init__(self, name, description, inputs=None, outputs=None, parameters=None):
        self.name = name
        self.description = description

        if inputs is not None:
            self.inputs = inputs

        if outputs is not None:
            self.outputs = outputs

        if parameters is not None:
            self.parameters = parameters

    def get_name(self):
        return self.name

    def get_description(self):
        return self.description

    def get_inputs(self):
        return self.inputs

    def get_outputs(self):
        return self.outputs

    def get_parameters(self):
        return self.parameters