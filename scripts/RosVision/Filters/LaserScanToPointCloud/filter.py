from ..filter import Filter
from ..filter_descriptor import FilterDescriptor
from ..io_descriptor import IODescriptor
from ...IO.point_cloud import PointCloud
from ...IO.laser_scan import LaserScan
import pcl

class LaserScanToPointCloud(Filter):
    descriptor = FilterDescriptor("LaserScanToPointCloud", "Converts a LaserScan to a PointCloud",
                                  inputs=[IODescriptor("scan", "Input LaserScan.", LaserScan)],
                                  outputs=[IODescriptor("cloud", "Output PointCloud.", PointCloud)])

    def initialize(self):
        print "Init %s" % self.name

    def execute(self, time=0):
        scan = self.get_input("scan")
        if scan:
            points = []
            for p in scan.get_laser_scan().points:
                points.append((p[0], p[1], 0))

            pc = pcl.PointCloud()
            pc.from_list(points)

            output = PointCloud(pc)
            scan.copy_header(output)

            self.set_output("cloud", output)