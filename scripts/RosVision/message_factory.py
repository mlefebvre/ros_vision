import ros_vision.msg
from io_manager import IOManager


class MessageFactory:
    def __init__(self):
        pass

    @staticmethod
    def create_workspace_message_from_workspace(workspace):
        w = ros_vision.msg.Workspace()
        w.input_topics = []
        w.filter_groups = []

        if workspace.name is None:
            w.name = ""
        else:
            w.name = workspace.name

        for filter_chain_inputs in workspace.input_topics:
            for input_topic in filter_chain_inputs:
                input = ros_vision.msg.IODescriptor()
                input.topic, input.type = input_topic
                w.input_topics.append(input)

        for filter_group_name in workspace.groups:
            fg = ros_vision.msg.FilterGroup()
            fg.name = filter_group_name[1:]
            fg.filters = workspace.groups[filter_group_name].filters.values()
            w.filter_groups.append(fg)

        return w

    @staticmethod
    def create_filter_message_from_descriptor(descriptor):
        filter = ros_vision.msg.Filter()
        filter.name = descriptor.get_name()
        filter.type = descriptor.get_name()
        filter.description = descriptor.description

        for i in descriptor.get_inputs():
            d = ros_vision.msg.IODescriptor()
            name = i.get_name()
            d.name = name
            d.type = str(i.get_io_type().get_ros_type()._type)
            d.topic = IOManager().format_topic_name(filter.name + "/" + name)
            filter.inputs.append(d)

        for o in descriptor.get_outputs():
            d = ros_vision.msg.IODescriptor()
            name = o.get_name()
            d.name = name
            d.type = str(o.get_io_type().get_ros_type()._type)
            d.topic = IOManager().format_topic_name(filter.name + "/" + name)
            filter.outputs.append(d)

        for p in descriptor.get_parameters():
            parameter = ros_vision.msg.Parameter()
            parameter.name = p.get_name()
            parameter.description = p.get_description()
            parameter.type = p.get_type().__name__
            parameter.default = str(p.get_default_value())
            parameter.min = str(p.get_min_value())
            parameter.max = str(p.get_max_value())
            filter.parameters.append(parameter)

        return filter

    @staticmethod
    def create_filter_message_from_filter(f):
        filter = MessageFactory.create_filter_message_from_descriptor(f.descriptor)
        filter.name = f.name

        for idx, i in enumerate(f.descriptor.get_inputs()):
            filter.inputs[idx].topic = IOManager().format_topic_name(f.get_io_name(filter.inputs[idx].name))

        for idx, o in enumerate(f.descriptor.get_outputs()):
            filter.outputs[idx].topic = IOManager().format_topic_name(f.get_io_name(filter.outputs[idx].name))

        return filter