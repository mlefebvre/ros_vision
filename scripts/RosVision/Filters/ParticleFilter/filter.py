from ..filter import Filter
from ..filter_descriptor import FilterDescriptor
from ..parameter_descriptor import ParameterDescriptor
from ..io_descriptor import IODescriptor
from ...IO.image import Image
import cv2
import numpy as np


class ParticleFilter(Filter):
    descriptor = FilterDescriptor("ParticleFilter", "Remove small particles from the image.",
                                  inputs=[IODescriptor("input", "Input grayscale image.", Image)],
                                  outputs=[IODescriptor("output", "Filtered image.", Image)],
                                  parameters=[ParameterDescriptor("kernel_height", "Kernel Height", int, 10, 1, 255),
                                              ParameterDescriptor("kernel_width", "Kernel Width", int, 10, 1, 255),
                                              ParameterDescriptor("min_area", "Minimum blob area", int, 1, 0, 3200)
                                              ])

    def initialize(self):
        print "Init %s" % self.name

    def execute(self, time=0):
        im = self.get_input("input")
        if im:
            min_area = self.get_param("min_area")
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.get_param("kernel_width"), self.get_param("kernel_height")))
            im2 = cv2.erode(im.get_image(), kernel)
            contours, _ = cv2.findContours(im2, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            im3 = np.zeros(im2.shape, np.uint8)
            for contour in contours:
                area = np.abs(cv2.contourArea(contour))
                if area > min_area:
                    cv2.drawContours(im3, [contour], -1, (255, 255, 255), thickness=-1)

            output = Image(im3)
            im.copy_header(output)
            self.set_output("output", output)