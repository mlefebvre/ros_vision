from ..filter import Filter
from ..filter_descriptor import FilterDescriptor
from ..io_descriptor import IODescriptor
from ...IO.point_cloud import PointCloud
import cv2
import pcl

class PCProject2D(Filter):
    descriptor = FilterDescriptor("PCProject2D", "Projects a point cloud to the x-y plane",
                                  inputs=[IODescriptor("input", "3D point cloud.", PointCloud)],
                                  outputs=[IODescriptor("output", "2D point cloud.", PointCloud)])

    def initialize(self):
        print "Init %s" % self.name

    def execute(self, time=0):
        pc = self.get_input("input")
        if pc:
            points = []
            for p in pc.get_point_cloud().to_list():
                points.append((p[0], p[1], 0))

            pc2 = pcl.PointCloud()
            pc2.from_list(points)

            output = PointCloud(pc2)
            pc.copy_header(output)

            self.set_output("output", output)