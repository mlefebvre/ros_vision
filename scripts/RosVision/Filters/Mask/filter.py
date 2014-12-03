from ..filter import Filter
from ..filter_descriptor import FilterDescriptor
from ..parameter_descriptor import ParameterDescriptor
from ..io_descriptor import IODescriptor
from ...IO.image import Image
import cv2

class MaskFilter(Filter):
    descriptor = FilterDescriptor("Mask", "Applies a bitwise mask to an image.",
                                  inputs=[IODescriptor("input", "Input image.", Image),
                                          IODescriptor("mask", "Input mask", Image)],
                                  outputs=[IODescriptor("output", "Resized image.", Image)])

    def initialize(self):
        print "Init %s" % self.name

    def execute(self, time=0):
        im, mask = self.get_input("input", "mask")
        if im and mask:
            im2 = Image(cv2.bitwise_and(im.get_image(), im.get_image(), mask=mask.get_image()))
            im.copy_header(im2)
            self.set_output("output", im2)