#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
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
        self._pub = rospy.Publisher("/imu", Imu, queue_size=10)
        self._rate = rospy.Rate(50)
        self._r_orientation = Quaternion()
        self._p_orientation = Quaternion()
        self._imu_data = Imu()
        self._p_imu = Imu()

    def e2q_cb(self, data):
        self._r_orientation = data.orientation
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

    def publisher(self):
        while not rospy.is_shutdown():
            self._p_orientation.w = self._offset_q.w * self._r_orientation.w - self._offset_q.x * self._r_orientation.x - self._offset_q.y * self._r_orientation.y - self._offset_q.z * self._r_orientation.z
            self._p_orientation.x = self._offset_q.w * self._r_orientation.x + self._offset_q.x * self._r_orientation.w + self._offset_q.y * self._r_orientation.z - self._offset_q.z * self._r_orientation.y
            self._p_orientation.y = self._offset_q.w * self._r_orientation.y - self._offset_q.x * self._r_orientation.z + self._offset_q.y * self._r_orientation.w + self._offset_q.z * self._r_orientation.x
            self._p_orientation.z = self._offset_q.w * self._r_orientation.z + self._offset_q.x * self._r_orientation.y - self._offset_q.y * self._r_orientation.x + self._offset_q.z * self._r_orientation.w
            self._imu_data.orientation = self._p_orientation
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
