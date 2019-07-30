#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from sensor_msgs.msg import Imu
import scipy

accel = Float32()


def imu_cb(data):
    global accel
    accel = data.linear_acceleration.x

def test():
    global accel
    rospy.init_node("imu_test")
    sub = rospy.Subscriber("/android/imu", Imu, imu_cb)
    pub = rospy.Publisher("/vel_x", Float32, queue_size=10)
    rate = rospy.Rate(50)
    new_data = old_data = 0.0
    final_value = 0
    while not rospy.is_shutdown():
        new_data = accel
        if type(new_data) == float:
            area = 0.02 * (old_data + (0.5 * (new_data - old_data)))
            final_value += area
            pub.publish(final_value)
            old_data = new_data
        rospy.loginfo(new_data)
        rate.sleep()

if __name__ == "__main__":
    try:
        test()
    except rospy.ROSInterruptException:
        pass
