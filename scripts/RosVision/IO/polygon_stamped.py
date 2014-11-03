from io_object_stamped import IOObjectStamped
import geometry_msgs.msg as m


class PolygonStamped(IOObjectStamped):
    def _to_ros_msg(self):
        msg = m.PolygonStamped()
        for x, y, z in self.value:
            p2 = m.Point32()
            p2.x = x
            p2.y = y
            p2.z = z
            msg.polygon.points.append(p2)
        msg.header = self._get_header()
        return msg

    def _from_ros_msg(self, data):
        self._set_header(data.header)
        value = []
        for p in data.polygon.points:
            value.append((p.x, p.y, p.z))
        return value

    @staticmethod
    def get_ros_type():
        return m.PolygonStamped

    def get_points(self):
        if type(self.value) == self.get_ros_type():
            self.value = self._from_ros_msg(self.value)
        return self.value


