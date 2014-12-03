from io_object_stamped import IOObjectStamped
import sensor_msgs.msg as m
import math


class Scan:
    def __init__(self, points, time_increment, scan_time, range_min, range_max):
        self.points = points
        self.time_increment = time_increment
        self.scan_time = scan_time
        self.range_min = range_min
        self.range_max = range_max


class LaserScan(IOObjectStamped):

    def _to_ros_msg(self):
        val = self.value
        msg = m.LaserScan()
        msg.header = self._get_header()
        msg.time_increment = val.time_increment
        msg.scan_time = val.scan_time
        msg.range_min = val.range_min
        msg.range_max = val.range_max

        for p in val.points:
            d = math.sqrt(p[0]*p[0] + p[1]*p[1])
            angle = math.atan2(p[1], p[0])
            if angle < msg.angle_min:
                msg.angle_min = angle
            elif angle > msg.angle_max:
                msg.angle_max = angle
            msg.ranges.append(d)
        msg.angle_increment = (msg.angle_max - msg.angle_min) / len(msg.ranges)

        return msg

    def _from_ros_msg(self, msg):
        self._set_header(msg.header)
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            points.append((math.cos(angle) * r, math.sin(angle) * r))
            angle += msg.angle_increment

        scan = Scan(points, msg.time_increment, msg.scan_time, msg.range_min, msg.range_max)
        return scan

    @staticmethod
    def get_ros_type():
        return m.LaserScan

    def get_laser_scan(self):
        if type(self.value) == self.get_ros_type():
            self.value = self._from_ros_msg(self.value)
        return self.value


