from ..filter import Filter


class ResizeFilter(Filter):
    def initialize(self):
        print "Init %s" % self.name
        print self.get_param("width")

    def execute(self, time=0):
        print self.name
        import time
        time.sleep(3)