from ..filter import Filter
from ..filter_descriptor import FilterDescriptor
from ..parameter_descriptor import ParameterDescriptor
from ..io_descriptor import IODescriptor
from ...IO.image import Image
import cv2

class Gray2BGRFilter(Filter):
    descriptor = FilterDescriptor("Gray2BGR", "Convert a gray image to the BGR color space.",
                                  inputs=[IODescriptor("input", "Gray image.", Image)],
                                  outputs=[IODescriptor("output", "BGR image.", Image)])

    def initialize(self):
        print "Init %s" % self.name

    def execute(self, time=0):
        im = self.get_input("input")
        if im:
            im2 = Image(cv2.cvtColor(im.get_image(), cv2.COLOR_GRAY2BGR))
            self.set_output("output", im2)