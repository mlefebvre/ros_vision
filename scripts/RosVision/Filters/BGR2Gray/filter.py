from ..filter import Filter
from ..filter_descriptor import FilterDescriptor
from ..parameter_descriptor import ParameterDescriptor
from ..io_descriptor import IODescriptor
from ...IO.image import Image
import cv2

class BGR2GrayFilter(Filter):
    descriptor = FilterDescriptor("BGR2Gray", "Convert a BGR image to the gray color space.",
                                  inputs=[IODescriptor("input", "BGR image.", Image)],
                                  outputs=[IODescriptor("output", "Gray image.", Image)])

    def initialize(self):
        print "Init %s" % self.name

    def execute(self, time=0):
        im = self.get_input("input")
        if im:
            im2 = Image(cv2.cvtColor(im.get_image(), cv2.COLOR_BGR2GRAY))
            im.copy_header(im2)
            self.set_output("output", im2)