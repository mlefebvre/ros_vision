from ..filter import Filter
from ..filter_descriptor import FilterDescriptor
from ..parameter_descriptor import ParameterDescriptor
from ..io_descriptor import IODescriptor
from ...IO.image import Image
import cv2

class EmptyFilter(Filter):
    descriptor = FilterDescriptor("Empty", "Does nothing",
                                  inputs=[IODescriptor("input", "Input", Image)],
                                  outputs=[IODescriptor("output", "Output", Image)])

    def initialize(self):
        print "Init %s" % self.name

    def execute(self, time=0):
        im = self.get_input("input")
        if im:
            im2 = Image(im.get_image())
            im.copy_header(im2)
            self.set_output("output", im2)