from ..filter import Filter
from ..filter_descriptor import FilterDescriptor
from ..parameter_descriptor import ParameterDescriptor
from ..io_descriptor import IODescriptor
from ...IO.image import Image
import cv2

class DilateFilter(Filter):
    descriptor = FilterDescriptor("Dilate", "Dilates an image by using a specific structuring element.",
                                  inputs=[IODescriptor("input", "Source image.", Image)],
                                  outputs=[IODescriptor("output", "Output image.", Image)],
                                  parameters=[ParameterDescriptor("element_shape", "Element shape", str, "rect", ["rect", "ellipse", "cross"]),
                                              ParameterDescriptor("element_size", "Size of the structuring element.", int, 3, 0, 50),
                                              ParameterDescriptor("iterations", "Number of times erosion is applied.", int, 1, 0, 20)
                                              ])
    element_shapes = {"rect": cv2.MORPH_RECT,
                       "ellipse": cv2.MORPH_ELLIPSE,
                       "cross": cv2.MORPH_CROSS}

    def initialize(self):
        print "Init %s" % self.name

    def execute(self, time=0):
        im = self.get_input("input")
        if im:
            element_shape = self.element_shapes[self.get_param("element_shape")]
            element_size = self.get_param("element_size")
            element = cv2.getStructuringElement(element_shape, (element_size, element_size))
            # ITERATIONS
            im2 = Image(cv2.dilate(im.get_image(), element))
            im.copy_header(im2)
            self.set_output("output", im2)