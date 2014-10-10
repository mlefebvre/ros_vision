from ..filter import Filter
from ..filter_descriptor import FilterDescriptor
from ..parameter_descriptor import ParameterDescriptor
from ..io_descriptor import IODescriptor
from ...IO.image import Image
import cv2

class BilateralFilter(Filter):
    descriptor = FilterDescriptor("BilateralFilter", "Applies the bilateral filter to an image.",
                                  inputs=[IODescriptor("input", "Source image.", Image)],
                                  outputs=[IODescriptor("output", "Filtered image.", Image)],
                                  parameters=[ParameterDescriptor("d", "Diameter of each pixel neighborhood that is used during filtering. If it is non-positive, it is computed from sigmaSpace .", int, 3, 0, 1024),
                                              ParameterDescriptor("sigma_color", "Filter sigma in the color space. A larger value of the parameter means that farther colors within the pixel neighborhood (see sigmaSpace ) will be mixed together, resulting in larger areas of semi-equal color.", int, 150, 0, 255),
                                              ParameterDescriptor("sigma_space", "Filter sigma in the coordinate space. A larger value of the parameter means that farther pixels will influence each other as long as their colors are close enough (see sigmaColor ). When d>0 , it specifies the neighborhood size regardless of sigmaSpace . Otherwise, d is proportional to sigmaSpace.", int, 150, 0, 255)
                                              ])

    def initialize(self):
        print "Init %s" % self.name

    def execute(self, time=0):
        im = self.get_input("input")
        if im:
            im2 = Image(cv2.bilateralFilter(im.get_image(), self.get_param("d"), self.get_param("sigma_color"), self.get_param("sigma_space")))
            self.set_output("output", im2)