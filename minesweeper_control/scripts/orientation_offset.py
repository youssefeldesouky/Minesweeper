#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
import math


class Converter(object):
    def __init__(self):
        rospy.init_node("offseter")
        self._offset_str = rospy.get_param("/imu/offset", [0.0, 0.0, 0.0])
        # self._offset_list = self._offset_str[1:len(self._offset_str)]
        # self._offset_list = [float(x) for x in str.split(self._offset_str, ', ')]
        self._offset_q = self.calculate_offset(self._offset_str)
        self._sub = rospy.Subscriber("/android/imu", Imu, self.e2q_cb)
        self._pub = rospy.Publisher("imu/data", Imu, queue_size=10)
        self._rate = rospy.Rate(50) # it was 50Hz
        self._r_orientation = Quaternion()
        self._p_orientation = Quaternion()
        self._imu_data = Imu()
        self._p_imu = Imu()
        self._single_count = 0
        self._init_imu = Imu()
        self._off_euler = []
        self._off_quat = Quaternion()

    def e2q_cb(self, data):
        self._r_orientation = data.orientation
        if not self._single_count:
            self._init_imu = data
            self._single_count += 1
        self._imu_data = data

    def calculate_offset(self, offset):
        x = math.radians(offset[0])
        y = math.radians(offset[1])
        z = math.radians(offset[2])
        cz = math.cos(z / 2)
        sz = math.sin(z / 2)
        cy = math.cos(y / 2)
        sy = math.sin(y / 2)
        cx = math.cos(x / 2)
        sx = math.sin(x / 2)
        q = Quaternion()
        q.w = cz * cy * cx + sz * sy * sx
        q.x = cz * cy * sx - sz * sy * cx
        q.y = sz * cy * sx + cz * sy * cx
        q.z = sz * cy * cx - cz * sy * sx
        return q

    def quat_to_euler(self, quat):
        x = math.atan2(2 * (quat.w * quat.x + quat.y * quat.z), 1 - 2 * (quat.x ** 2 + quat.y ** 2))
        y = math.asin(2 * (quat.w * quat.y - quat.z * quat.x))
        z = math.atan2(2 * (quat.w * quat.z + quat.x * quat.y), 1 - 2 * (quat.y ** 2 + quat.z ** 2))
        return (x, y, z)



    def publisher(self):
        while not rospy.is_shutdown():
            self._p_orientation.w = self._offset_q.w * self._r_orientation.w - self._offset_q.x * self._r_orientation.x - self._offset_q.y * self._r_orientation.y - self._offset_q.z * self._r_orientation.z
            self._p_orientation.x = self._offset_q.w * self._r_orientation.x + self._offset_q.x * self._r_orientation.w + self._offset_q.y * self._r_orientation.z - self._offset_q.z * self._r_orientation.y
            self._p_orientation.y = self._offset_q.w * self._r_orientation.y - self._offset_q.x * self._r_orientation.z + self._offset_q.y * self._r_orientation.w + self._offset_q.z * self._r_orientation.x
            self._p_orientation.z = self._offset_q.w * self._r_orientation.z + self._offset_q.x * self._r_orientation.y - self._offset_q.y * self._r_orientation.x + self._offset_q.z * self._r_orientation.w
            self._imu_data.orientation = self._p_orientation
            self._imu_data.orientation_covariance =[1e-3, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 1e-3]
            self._imu_data.angular_velocity_covariance = [1e-3, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 1e-3]
            self._imu_data.linear_acceleration_covariance = [1e-3, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 1e-3]
            self._imu_data.header.frame_id = "imu_link"
            self._imu_data.header.stamp = rospy.get_rostime()
            self._pub.publish(self._imu_data)
            self._rate.sleep()

    def init_publisher(self):
        self._off_euler = self.quat_to_euler(self._init_imu.orientation)

        self._off_euler *= -1
        rospy.loginfo(self._off_euler)
        #self._off_quat = self.calculate_offset(self._off_euler

        while not rospy.is_shutdown():
            self._off_euler = self.quat_to_euler(self._init_imu.orientation)

            self._off_euler *= -1
            rospy.loginfo(self._imu_data)
            self._p_orientation.w = self._off_quat.w * self._r_orientation.w - self._off_quat.x * self._r_orientation.x - self._off_quat.y * self._r_orientation.y - self._off_quat.z * self._r_orientation.z
            self._p_orientation.x = self._off_quat.w * self._r_orientation.x + self._off_quat.x * self._r_orientation.w + self._off_quat.y * self._r_orientation.z - self._off_quat.z * self._r_orientation.y
            self._p_orientation.y = self._off_quat.w * self._r_orientation.y - self._off_quat.x * self._r_orientation.z + self._off_quat.y * self._r_orientation.w + self._off_quat.z * self._r_orientation.x
            self._p_orientation.z = self._off_quat.w * self._r_orientation.z + self._off_quat.x * self._r_orientation.y - self._off_quat.y * self._r_orientation.x + self._off_quat.z * self._r_orientation.w
            self._imu_data.orientation = self._p_orientation
            self._imu_data.orientation_covariance =[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self._imu_data.angular_velocity_covariance = [1e-06, 0.0, 0.0, 0.0, 1e-06, 0.0, 0.0, 0.0, 1e-06]
            self._imu_data.linear_acceleration_covariance = [1e-06, 0.0, 0.0, 0.0, 1e-06, 0.0, 0.0, 0.0, 1e-06]
            self._imu_data.header.frame_id = "imu_link"
            self._imu_data.header.stamp = rospy.get_rostime()
            self._pub.publish(self._imu_data)
            self._rate.sleep()


def main():
    p = Converter()

    p.publisher()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
