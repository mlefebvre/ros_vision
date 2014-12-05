from ..filter import Filter
from ..filter_descriptor import FilterDescriptor
from ..parameter_descriptor import ParameterDescriptor
from ..io_descriptor import IODescriptor
from ...IO.image import Image
from ...IO.point_cloud import PointCloud
import cv2

class BitwiseToPointCloud(Filter):
    descriptor = FilterDescriptor("BitwiseToPointCloud", "Converts a bitwise image to a point cloud",
                                  inputs=[IODescriptor("input", "Input image.", Image)],
                                  outputs=[IODescriptor("output", "Output point cloud.", Image)])

    def initialize(self):
        print "Init %s" % self.name

    def execute(self, time=0):
        im = self.get_input("input")
        if im:



            self.set_output("output", out)