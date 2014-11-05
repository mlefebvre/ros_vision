from ..filter import Filter
from ..filter_descriptor import FilterDescriptor
from ..parameter_descriptor import ParameterDescriptor
from ..io_descriptor import IODescriptor
from ...IO.image import Image
import cv2

class BitwiseOrFilter(Filter):
    descriptor = FilterDescriptor("BitwiseOr", "Applies a bitwise or mask to an image.",
                                  inputs=[IODescriptor("input1", "First input image.", Image),
                                          IODescriptor("input2", "Second input image", Image)],
                                  outputs=[IODescriptor("output", "Resized image.", Image)])

    def initialize(self):
        print "Init %s" % self.name

    def execute(self, time=0):
        im1 = self.get_input("input1")
        im2 = self.get_input("input2")
        if im1 and im2:
            out = Image(cv2.bitwise_or(im1.get_image(), im2.get_image()))
            im1.copy_header(out)
            self.set_output("output", out)