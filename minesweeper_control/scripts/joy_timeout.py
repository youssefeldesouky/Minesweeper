#!/usr/bin/env python
import rospy
import rostopic
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

def timeout():
    rospy.init_node('joy_timeout')
    rate = rospy.Rate(5)
    joy_rate = rostopic.ROSTopicHz(-1)
    timeout_list = []
    decision = True
    counter = 0
    joy_sub = rospy.Subscriber('/joy', Joy, joy_rate.callback_hz, callback_args="/joy")
    joy_timeout_pub = rospy.Publisher('/joy_timeout', Bool, queue_size=1)
    while not rospy.is_shutdown():
        x = bool((joy_rate.get_hz('/joy')))
        if not x:
            timeout_list.insert(counter, x)
        if len(timeout_list) == 6:
            decision = False
            del timeout_list[:]
            counter = 0
        if x:
            decision = True

        joy_timeout_pub.publish(decision)
        counter += 1
        rate.sleep()

if __name__ == "__main__":
    try:
        timeout()
    except rospy.ROSInterruptException:
        pass
