from io_object import IOObject
import geometry_msgs.msg as m


class Polygon(IOObject):
    def _to_ros_msg(self):
        msg = m.Polygon()
        for x, y, z in self.value:
            p2 = m.Point32()
            p2.x = x
            p2.y = y
            p2.z = z
            msg.points.append(p2)
        return msg

    def _from_ros_msg(self, data):
        value = []
        for p in data.points:
            value.append((p.x, p.y, p.z))
        return value

    @staticmethod
    def get_ros_type():
        return m.Polygon

    def get_points(self):
        if type(self.value) == self.get_ros_type():
            self.value = self._from_ros_msg(self.value)
        return self.value


