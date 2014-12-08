from ..filter import Filter
from ..filter_descriptor import FilterDescriptor
from ..parameter_descriptor import ParameterDescriptor
from ..io_descriptor import IODescriptor
from ...IO.image import Image
from ...IO.point_cloud import PointCloud
import cv2
import numpy as np
import pcl

class BitwiseToPointCloud(Filter):
    descriptor = FilterDescriptor("BitwiseToPointCloud", "Converts a bitwise image to a point cloud",
                                  inputs=[IODescriptor("input", "Input image.", Image)],
                                  outputs=[IODescriptor("cloud", "Output point cloud.", PointCloud)],
                                  parameters=[ParameterDescriptor("camera_dist", "Distance between the camera and the image", float, 0.75, 0, 3),
                                              ParameterDescriptor("real_size_x", "Real size of the width of the image (in meters)", float, 1.76, 0, 20),
                                              ParameterDescriptor("real_size_y", "Real size of the height of the image (in meters)", float, 1.54, 0, 20)])

    def initialize(self):
        print "Init %s" % self.name

    def execute(self, time=0):
        im = self.get_input("input")
        if im:
            points = []
            im1 = im.get_image()

            width = float(im1.shape[1])
            height = float(im1.shape[0])
            real_size_x = self.get_param("real_size_x")
            real_size_y = self.get_param("real_size_y")
            camera_dist = self.get_param("camera_dist")

            for y in xrange(0, im1.shape[0]):
                for x in xrange(0, im1.shape[1]):
                    val = im1[y, x]
                    if val > 0:
                        points.append(((y/height)*real_size_y + camera_dist, (x/width) * real_size_x - real_size_x/2.0, 0))

            pc = pcl.PointCloud()
            pc.from_list(points)

            output = PointCloud(pc)
            im.copy_header(output)
            output.frame = "/laser"

            self.set_output("cloud", output)