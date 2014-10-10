from ..filter import Filter
from ..filter_descriptor import FilterDescriptor
from ..parameter_descriptor import ParameterDescriptor
from ..io_descriptor import IODescriptor
from ...IO.image import Image
import cv2

class ResizeFilter(Filter):
    descriptor = FilterDescriptor("Resize", "Resizes an image.",
                                  inputs=[IODescriptor("input", "Input image.", Image)],
                                  outputs=[IODescriptor("output", "Resized image.", Image)],
                                  parameters=[ParameterDescriptor("width", "New width.", int, 800, 1, 4096),
                                              ParameterDescriptor("height", "New height.", int, 600, 1, 4096)
                                              ])

    def initialize(self):
        print "Init %s" % self.name

    def execute(self, time=0):
        im = self.get_input("input")
        if im:
            im2 = Image(cv2.resize(im.get_image(), (self.get_param("width"), self.get_param("height"))))
            self.set_output("output", im2)