#!/usr/bin/python

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion
from rift import PyRift


class OculusDriver(object):
    def __init__(self):
        rospy.init_node('openhmd_ros')
        self.pub = rospy.Publisher('/openhmd/pose', Pose)
        self.hmd = None

    def start(self):
        self.hmd = PyRift()
        print self.hmd.getDeviceInfo()
        if self.hmd:
            self.run()

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            d = self.hmd.poll()
            if len(d) != 4:
            	continue

            pose = Pose()
            pose.orientation = Quaternion(d[0], d[1], d[2], d[3])

            euler = euler_from_quaternion(d)
            pose.position = Point(euler[0], euler[1], euler[2])

            self.pub.publish(pose)
            r.sleep()

if __name__ == "__main__":
    driver = OculusDriver()
    driver.start()