import subprocess
import rosnode
import rospy


class Node:
    def __init__(self, package, type, name):
        self.package = package
        self.type = type
        self.name = name
        self.process = None

    def run(self):
        self.process = subprocess.Popen('rosrun %s %s __name:=%s' % (self.package, self.type, self.name), shell=True)

    def kill(self):
        rospy.loginfo("Killing /%s" % self.name)
        rosnode.kill_nodes([self.name])


