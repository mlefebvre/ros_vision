from io_object_stamped import IOObjectStamped
import ros_vision.msg as m
from polygon import Polygon

class PolygonListStamped(IOObjectStamped):
    def _to_ros_msg(self):
        msg = m.PolygonListStamped()
        for poly in self.value:
            msg.polygons.append(poly.to_ros_msg())
        msg.header = self._get_header()
        return msg

    def _from_ros_msg(self, data):
        self._set_header(data.header)
        value = []
        for p in data.polygons:
            value.append(Polygon(p))
        return value

    @staticmethod
    def get_ros_type():
        return m.PolygonListStamped

    def get_polygons(self):
        if type(self.value) == self.get_ros_type():
            self.value = self._from_ros_msg(self.value)
        return self.value


