from ..filter import Filter
from ..filter_descriptor import FilterDescriptor
from ..parameter_descriptor import ParameterDescriptor
from ..io_descriptor import IODescriptor
from ...IO.image import Image
from ...IO.polygon_list_stamped import PolygonListStamped
from ...IO.polygon import Polygon
import os
import cv2

class CascadeClassifierFilter(Filter):
    descriptor = FilterDescriptor("CascadeClassifier", "Haar Feature-based Cascade Classifier for Object Detection.",
                                  inputs=[IODescriptor("input", "Grayscale image.", Image)],
                                  outputs=[IODescriptor("faces", "Detected faces", PolygonListStamped)],
                                  parameters=[ParameterDescriptor("classifier", 'The classifier file', str, 'haarcascade_frontalface_alt.xml')]
                                  )
    face_cascade = None

    def initialize(self):
        print "Init %s" % self.name
        classifier = self.get_param("classifier")
        path = classifier
        if not os.path.isfile(path):
            path = os.path.join(os.path.dirname(os.path.realpath(__file__)), classifier)
            if not os.path.isfile(path):
                path = os.path.join('/', 'usr', 'share', 'opencv', 'haarcascades', classifier)

        self.cascade = cv2.CascadeClassifier()
        self.cascade.load(path)

    def execute(self, time=0):
        im = self.get_input("input")
        if im:
            faces = self.cascade.detectMultiScale(im.get_image(), 1.3, 5)
            polygons = []
            for x, y, w, h in faces:
                p = Polygon([(x, y, 0), (x + w, y, 0), (x + w, y + h, 0), (x, y + h, 0)])
                polygons.append(p)
            output = PolygonListStamped(polygons)
            im.copy_header(output)
            self.set_output("faces", output)