from ..filter import Filter
from ..filter_descriptor import FilterDescriptor
from ..parameter_descriptor import ParameterDescriptor
from ..io_descriptor import IODescriptor
from ...IO.image import Image
import cv2
import numpy as np


class PerspectiveTransform(Filter):
    descriptor = FilterDescriptor("PerspectiveTransform", "Applies a perspective transform to an image",
                                  inputs=[IODescriptor("input", "Source image.", Image)],
                                  outputs=[IODescriptor("output", "Transformed image.", Image)],
                                  parameters=[ParameterDescriptor("m00", "m00", float, 3.98252108e+00, 0, 100),
                                              ParameterDescriptor("m01", "m01", float, 1.86034722e+00, 0, 100),
                                              ParameterDescriptor("m02", "m02", float, -8.99071360e+02, 0, 100),
                                              ParameterDescriptor("m10", "m10", float, 2.22044605e-15, 0, 100),
                                              ParameterDescriptor("m11", "m11", float, 6.59120589e+00, 0, 100),
                                              ParameterDescriptor("m12", "m12", float, -1.25232912e+03, 0, 100),
                                              ParameterDescriptor("m20", "m20", float, -1.18465990e-05, 0, 100),
                                              ParameterDescriptor("m21", "m21", float, 6.19724197e-03, 0, 100),
                                              ParameterDescriptor("m22", "m22", float, 1.0, 0, 100)])
    def initialize(self):
        print "Init %s" % self.name

    def execute(self, time=0):
        im = self.get_input("input")
        if im:
            im1 = im.get_image()
            matrix = np.array([[self.get_param("m00"), self.get_param("m01"), self.get_param("m02")],
                               [self.get_param("m10"), self.get_param("m11"), self.get_param("m12")],
                               [self.get_param("m20"), self.get_param("m21"), self.get_param("m22")]])
            im2 = Image(cv2.warpPerspective(im1, matrix, (im1.shape[1], im1.shape[0])))
            im.copy_header(im2)
            self.set_output("output", im2)