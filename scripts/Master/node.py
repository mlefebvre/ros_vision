import subprocess
import rosnode
import rospy
import rospkg
import os


class Node:
    def __init__(self, package, type, name, profile=False):
        self.package = package
        self.type = type
        self.name = name
        self.process = None
        self.profile = profile

    def run(self):
        if self.profile:
            rospack = rospkg.RosPack()
            script_path = os.path.join(rospack.get_path(self.package), "scripts", self.type)
            command = "python -m cProfile -o %s %s __name:=%s" % ("~/profiler.txt", script_path, self.name)
            self.process = subprocess.Popen(command, shell=True)
        else:
            self.process = subprocess.Popen('rosrun %s %s __name:=%s' % (self.package, self.type, self.name), shell=True)

    def kill(self):
        rospy.loginfo("Killing /%s" % self.name)
        rosnode.kill_nodes([self.name])


