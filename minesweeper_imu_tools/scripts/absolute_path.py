#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math

class Prog(object):
    def __init__(self):
        rospy.init_node("line")
        self._sub = rospy.Subscriber("/odometry/filtered", Odometry, self.cb)
        self._pub = rospy.Publisher("line", Float64, queue_size=10)
        self._rate = rospy.Rate(50)
        self._x = 0.0
        self._y = 0.0
        self._s = 0.0

    def cb(self, data):
        self._x = data.pose.pose.position.x
        self._y = data.pose.pose.position.y
        self._s = math.sqrt(self._x ** 2 + self._y ** 2)

    def p(self):
        while not rospy.is_shutdown():
            self._pub.publish(self._s)
            self._rate.sleep()

if __name__ == "__main__":
    try:
        p = Prog()
        p.p()
    except rospy.ROSInterruptException:
        pass
