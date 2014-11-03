from ..filter import Filter
from ..filter_descriptor import FilterDescriptor
from ..parameter_descriptor import ParameterDescriptor
from ..io_descriptor import IODescriptor
from ...IO.image import Image
import cv2

class EqualizeHistogramFilter(Filter):
    descriptor = FilterDescriptor("EqualizeHistogram", "Equalizes the histogram of a grayscale image.",
                                  inputs=[IODescriptor("input", "Grayscale image.", Image)],
                                  outputs=[IODescriptor("output", "Equalized image.", Image)])

    def initialize(self):
        print "Init %s" % self.name

    def execute(self, time=0):
        im = self.get_input("input")
        if im:
            im2cv = im.get_image().copy()
            cv2.equalizeHist(im.get_image(), im2cv)
            im2 = Image(im2cv)
            im.copy_header(im2)
            self.set_output("output", im2)