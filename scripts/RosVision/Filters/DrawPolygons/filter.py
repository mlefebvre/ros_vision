from ..filter import Filter
from ..filter_descriptor import FilterDescriptor
from ..io_descriptor import IODescriptor
from ..parameter_descriptor import ParameterDescriptor
from ...IO.image import Image
import cv2
from ...IO.polygon_list_stamped import PolygonListStamped
import numpy

class DrawPolygonsFilter(Filter):
    descriptor = FilterDescriptor("DrawPolygons", "Draws polygons on an image",
                                  inputs=[IODescriptor("input", "Input image.", Image),
                                          IODescriptor("polygons", "Input polygons", PolygonListStamped)],
                                  outputs=[IODescriptor("output", "Output image.", Image)],
                                  parameters=[ParameterDescriptor("is_closed", "Flag indicating whether the drawn polylines are closed or not. If they are closed, the function draws a line from the last vertex of each curve to its first vertex.", bool, True),
                                              ParameterDescriptor("color_red", "Polyline red color.", int, 0, 0, 255),
                                              ParameterDescriptor("color_green", "Polyline green color.", int, 0, 0, 255),
                                              ParameterDescriptor("color_blue", "Polyline blue color.", int, 0, 0, 255),
                                              ParameterDescriptor("thickness", "Thickness of the polyline edges.", int, 1, 1, 20)
                                              ])

    def initialize(self):
        print "Init %s" % self.name

    def execute(self, time=0):
        im, polygons = self.get_input("input", "polygons")
        if im and polygons:
            im2cv = im.get_image().copy()
            pts = [numpy.array([[[x, y] for x,y,z in p.get_points()]], numpy.int32) for p in polygons.get_polygons()]
            cv2.polylines(im2cv, pts, self.get_param("is_closed"), (self.get_param("color_blue"), self.get_param("color_green"), self.get_param("color_red")), thickness=self.get_param("thickness"))
            im2 = Image(im2cv)
            im.copy_header(im2)
            self.set_output("output", im2)