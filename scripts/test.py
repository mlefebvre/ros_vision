from RosVision.IO.laser_scan import LaserScan
import rospy
import sensor_msgs.msg

rospy.init_node("test")
t = rospy.Publisher('/scan2', sensor_msgs.msg.LaserScan, queue_size=1, latch=True)
msg = rospy.wait_for_message('/scan', sensor_msgs.msg.LaserScan)

ls = LaserScan(msg)
print ls.get_laser_scan().points

m2 = ls.to_ros_msg()

print m2
print len(msg.ranges)

t.publish(m2)
print "OK"
#
#while not rospy.is_shutdown():
#     pass
#
#
