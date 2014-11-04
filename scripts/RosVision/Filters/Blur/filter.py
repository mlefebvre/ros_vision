from ..filter import Filter
from ..filter_descriptor import FilterDescriptor
from ..parameter_descriptor import ParameterDescriptor
from ..io_descriptor import IODescriptor
from ...IO.image import Image
import cv2

class BlurFilter(Filter):
    descriptor = FilterDescriptor("Blur", "Blurs an image using the normalized box filter.",
                                  inputs=[IODescriptor("input", "Source image.", Image)],
                                  outputs=[IODescriptor("output", "Blured image.", Image)],
                                  parameters=[ParameterDescriptor("ksize", "Blurring kernel size.", int, 3, 1, 1024)])

    def initialize(self):
        print "Init %s" % self.name

    def execute(self, time=0):
        im = self.get_input("input")
        if im:
            ksize = self.get_param("ksize")
            im2 = Image(cv2.blur(im.get_image(), (ksize, ksize)))
            im.copy_header(im2)
            self.set_output("output", im2)