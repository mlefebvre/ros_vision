from ..filter import Filter
from ..filter_descriptor import FilterDescriptor
from ..parameter_descriptor import ParameterDescriptor
from ..io_descriptor import IODescriptor
from ...IO.image import Image
import cv2
import numpy as np


class Psychedelic(Filter):
    descriptor = FilterDescriptor("Psychedelic", "Lysergic acid diethylamide",
                                  inputs=[IODescriptor("input", "Source image.", Image)],
                                  outputs=[IODescriptor("output", "Psychedelic image.", Image)],
                                  parameters=[ParameterDescriptor("image_count", "Image count", int, 10, 2, 100)])

    def initialize(self):
        print "Init %s" % self.name
        self.images = []

    def execute(self, time=0):
        im = self.get_input("input")
        if im:
            image = im.get_image()
            self.images.append(image)
            image_count = self.get_param("image_count")

            while len(self.images) > image_count:
                del self.images[0]

            for img in self.images:
                image = np.add(image, img)

            im2 = Image(image)
            im.copy_header(im2)
            self.set_output("output", im2)