#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import numpy as np


def signal_gen():
    pub = rospy.Publisher("input_signal", Twist, queue_size=10)
    rospy.init_node('input_generator_node', anonymous=True)
    rate = rospy.Rate(10)
    signal = Twist()
    i = 0
    while not rospy.is_shutdown():
        signal.linear.x = np.sin(2 * np.pi * i / 4000)
        signal.linear.y = np.cos(2 * np.pi * i / 4000)
        signal.linear.z = 0
        signal.angular.x = 0
        signal.angular.y = 0
        signal.angular.z = np.sin(2 * np.pi * i / 1000)
        pub.publish(signal)
        rospy.loginfo(signal)
        i = i + 1
        rate.sleep()


if __name__ == '__main__':
    try:
        signal_gen()
    except rospy.ROSInterruptException:
        pass
