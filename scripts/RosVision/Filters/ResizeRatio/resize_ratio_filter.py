from ..filter import Filter
from ..filter_descriptor import FilterDescriptor
from ..parameter_descriptor import ParameterDescriptor
from ..io_descriptor import IODescriptor
from ...IO.image import Image
import cv2

class ResizeRatioFilter(Filter):
    descriptor = FilterDescriptor("ResizeRatio", "Resizes an image.",
                                  inputs=[IODescriptor("input", "Input image.", Image)],
                                  outputs=[IODescriptor("output", "Resized image.", Image)],
                                  parameters=[ParameterDescriptor("width", "New width ratio.", int, 50, 1, 100),
                                              ParameterDescriptor("height", "New height ratio.", int, 50, 1, 100)
                                              ])

    def initialize(self):
        print "Init %s" % self.name
        print self.get_param("width")

    def execute(self, time=0):
        im = self.get_input("input")
        if im:
            im2 = Image(cv2.resize(im.get_value(), (0, 0), fx=self.get_param("width"), fy=self.get_param("height")))
            self.set_output("output", im2)