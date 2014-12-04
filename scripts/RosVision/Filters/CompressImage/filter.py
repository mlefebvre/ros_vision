from ..filter import Filter
from ..filter_descriptor import FilterDescriptor
from ..io_descriptor import IODescriptor
from ...IO.image import Image
from ...IO.compressed_image import CompressedImage


class CompressImage(Filter):
    descriptor = FilterDescriptor("CompressImage", "Compresses an image.",
                                  inputs=[IODescriptor("input", "Uncompressed image.", Image)],
                                  outputs=[IODescriptor("output", "Compressed image.", CompressedImage)])

    def initialize(self):
        print "Init %s" % self.name

    def execute(self, time=0):
        im = self.get_input("input")
        if im:
            im2 = CompressedImage(im.get_image())
            im.copy_header(im2)
            self.set_output("output", im2)