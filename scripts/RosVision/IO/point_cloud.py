from io_object_stamped import IOObjectStamped
import sensor_msgs.msg as m
import pcl
import struct


class PointCloud(IOObjectStamped):
    TYPES = {m.PointField.INT8: 'c',
             m.PointField.UINT8: 'b',
             m.PointField.INT16: 'h',
             m.PointField.UINT16: 'H',
             m.PointField.INT32: 'i',
             m.PointField.UINT32: 'I',
             m.PointField.FLOAT32: 'f',
             m.PointField.FLOAT64: 'd'}

    def _to_ros_msg(self):
        val = self.value
        msg = m.PointCloud2()
        msg.header = self._get_header()
        msg.height = val.height
        msg.width = val.width
        msg.fields = [m.PointField(name='x', offset=0, datatype=7, count=1),
                      m.PointField(name='y', offset=4, datatype=7, count=1),
                      m.PointField(name='z', offset=8, datatype=7, count=1)]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = val.width * msg.point_step
        msg.is_dense = val.is_dense
        msg.data = ''.join([struct.pack('fff', *p) for p in val.to_list()])
        return msg

    def _from_ros_msg(self, msg):
        self._set_header(msg.header)

        if msg.is_bigendian:
            data_format = ">"
        else:
            data_format = "<"

        for f in msg.fields:
            data_format += PointCloud.TYPES[f.datatype]
        step = struct.calcsize(data_format)

        points = []
        for i in range(0, len(msg.data), step):
            d = struct.unpack(data_format, msg.data[i:i+step])
            points.append(d[:3])

        pc = pcl.PointCloud()
        pc.from_list(points)
        return pc

    @staticmethod
    def get_ros_type():
        return m.PointCloud2

    def get_point_cloud(self):
        if type(self.value) == self.get_ros_type():
            self.value = self._from_ros_msg(self.value)
        return self.value


