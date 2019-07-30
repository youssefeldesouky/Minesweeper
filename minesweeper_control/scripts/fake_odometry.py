#!/usr/bin/env python
import rospy
from std_msgs.msg import *
import math


class Fake(object):
    def __init__(self):
        rospy.init_node("fake_odometry")
        self._joint_states = Float32MultiArray()
        self._max_ticks = 40
        self._left_cmd = 0
        self._right_cmd = 0
        self._l_sign = 1
        self._r_sign = 1
        self._l_pos = 0.0
        self._r_pos = 0.0
        self._l_ticks = 0
        self._r_ticks = 0
        self._left_sub = rospy.Subscriber("left_cmd", Int16, self.left_cb)
        self._right_sub = rospy.Subscriber("right_cmd", Int16, self.right_cb)
        self._signs_sub = rospy.Subscriber("sign", Int8MultiArray, self.sign_cb)
        self._js_pub = rospy.Publisher("joint_positions", Float32MultiArray, queue_size=10)
        self._rate = rospy.Rate(50)

    def left_cb(self, data):
        self._left_cmd = data.data

    def right_cb(self, data):
        self._right_cmd = data.data

    def sign_cb(self, data):
        self._l_sign = data.data[0]
        self._r_sign = data.data[1]

    def publish(self):
        while not rospy.is_shutdown():
            self._l_ticks += self._left_cmd / 20.0
            self._r_ticks += self._right_cmd / 20.0
            self._l_pos = (self._l_ticks/self._max_ticks) * math.pi * 2
            self._r_pos = (self._r_ticks / self._max_ticks) * math.pi * 2
            self._joint_states.data = [self._l_pos, self._r_pos, self._l_pos, self._r_pos]
            self._js_pub.publish(self._joint_states)
            self._rate.sleep()


if __name__ == "__main__":
    f = Fake()
    try:
        f.publish()
    except rospy.ROSInterruptException:
        pass
