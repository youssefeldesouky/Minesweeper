#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16MultiArray
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseWithCovariance, Pose
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int8

class Mine(object):
    def __init__(self):
        rospy.init_node("marker", anonymous=False)
        self._a = 0
        self._b = 0
        self._joy_sub = rospy.Subscriber("/joy", Joy, self.joy_callback)
        self._coil_sub = rospy.Subscriber("/coil_state", Int8, self.coil_callback)
        self._odom_sub = rospy.Subscriber("/coil_tf", Pose, self.odom_callback)
        self._rqt_mine_pub = rospy.Publisher("/rqt_mine_location", Int16MultiArray, queue_size=10)
        self._pub = rospy.Publisher("visualization_marker", Marker, queue_size=100)
        self._rate = rospy.Rate(10)
        self._marker_a = Marker()
        self._marker_b = Marker()
        self._marker_location_a = Pose()
        self._marker_location_b = Pose()
        self._detection_lock_a = False
        self._detection_lock_b = False
        self._id_counter_a = 0
        self._id_counter_b = 0
        self._rqt_mine_location = Int16MultiArray()
        self._rqt_mine_x = 0.0
        self._rqt_mine_y = 0.0
        self._rqt_mine_type = 0
        self._surface_error = []
        self._surface_error_counter = 0

    # comment out when testing with a real robot
    def joy_callback(self, data):
        self._a = data.buttons[0]
        self._b = data.buttons[1]
        if self._a == 0:
            self._detection_lock_a = False
        if self._b == 0:
            self._detection_lock_b = False

    def coil_callback(self, data):
        if data.data == 2:
            self._a = 1
            self._b = 0
        elif data.data == 1:
            self._b = 1
            self._a = 0
        else:
            self._a = self._b = 0
        if self._a == 0:
            self._detection_lock_a = False
        if self._b == 0:
            self._detection_lock_b = False
        rospy.loginfo(data.data)

    def odom_callback(self, data):
        self._rqt_mine_x = int((data.position.x))
        self._rqt_mine_y = int(-1 * (data.position.y))
        if self._a == 1 and self._b == 0 and not self._detection_lock_a:
            self._id_counter_a += 1
            self._marker_location_a = data
            self._rqt_mine_type = 1
            self._rqt_mine_location.data = (self._rqt_mine_x, self._rqt_mine_y, self._rqt_mine_type)
            self._rqt_mine_pub.publish(self._rqt_mine_location)
            self._detection_lock_a = True
        elif self._b and self._a == 0 and not self._detection_lock_b:
            self._id_counter_b += 1
            self._marker_location_b = data
            self._rqt_mine_type = -1
            self._rqt_mine_location.data = (self._rqt_mine_x, self._rqt_mine_y, self._rqt_mine_type)
            self._rqt_mine_pub.publish(self._rqt_mine_location)
            self._detection_lock_b = True

    def marker(self):
        while not rospy.is_shutdown():
            self._marker_a.header.frame_id = "/odom"
            self._marker_a.header.stamp = rospy.Time.now()
            self._marker_a.ns = "s_mines"
            self._marker_a.id = self._id_counter_a
            self._marker_a.type = self._marker_a.CUBE
            self._marker_a.action = self._marker_a.ADD
            self._marker_a.pose = self._marker_location_a
            self._marker_a.pose.orientation = Quaternion()
            self._marker_a.pose.orientation.w = 1.0
            self._marker_a.scale.x = 0.25
            self._marker_a.scale.y = 0.25
            self._marker_a.scale.z = 0.10
            self._marker_a.color.r = 1.0
            self._marker_a.color.g = 0.0
            self._marker_a.color.b = 0.0
            self._marker_a.color.a = 1.0
            self._marker_a.lifetime = rospy.Duration()

            self._marker_b.header.frame_id = "/odom"
            self._marker_b.header.stamp = rospy.Time.now()
            self._marker_b.ns = "b_mines"
            self._marker_b.id = self._id_counter_b
            self._marker_b.type = self._marker_b.CYLINDER
            self._marker_b.action = self._marker_b.ADD
            self._marker_b.pose = self._marker_location_b
            self._marker_b.pose.position.z = -0.15
            self._marker_b.pose.orientation = Quaternion()
            self._marker_b.pose.orientation.w = 1.0
            self._marker_b.scale.x = 0.25
            self._marker_b.scale.y = 0.25
            self._marker_b.scale.z = 0.10
            self._marker_b.color.r = 0.0
            self._marker_b.color.g = 1.0
            self._marker_b.color.b = 0.0
            self._marker_b.color.a = 1.0
            self._marker_b.lifetime = rospy.Duration()
            while self._pub.get_num_connections() < 1:
                pass
            if self._id_counter_a > 0:
                self._pub.publish(self._marker_a)

            if self._id_counter_b > 0:
                self._pub.publish(self._marker_b)
            self._rate.sleep()


def main():
    marker_server = Mine()
    rospy.loginfo(marker_server._a)
    marker_server.marker()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
