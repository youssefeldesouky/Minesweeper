#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
import tf2_ros

class Coil(object):
    def __init__(self):
        rospy.init_node("coil_tf_publisher")
        self.pub = rospy.Publisher("coil_tf", Pose, queue_size=1)
        self.rate = rospy.Rate(10)
        self.pose = Pose()

    def main(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        while not rospy.is_shutdown():
            try:
                self.t = self.tfBuffer.lookup_transform("odom", "coil_center_link", rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.rate.sleep()
                continue
            self.pose.position = self.t.transform.translation
            self.pose.orientation = self.t.transform.rotation
            self.pub.publish(self.pose)

if __name__ == "__main__":
    try:
        lis = Coil()
        lis.main()
    except rospy.ROSInterruptException:
        pass
