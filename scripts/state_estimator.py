#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import numpy as np

input_data = [0, 0, 0]


def callback_data(data):
    print(data)
    # TODO POBRAC ZASZUMIONĄ ODOMETRIE I ZAIMPLEMENTOWAĆ FILTR UŚREDNIAJĄCY
    global input_data


def main():
    odometry_pub = rospy.Publisher('estimated_odometry_pub', Odometry, queue_size=10)
    rospy.Subscriber("/noised_odometry_pub", Odometry, callback_data)
    rospy.init_node('state_estimator_node', anonymous=True)
    rate = rospy.Rate(10)
    i = 0
    delta_t = 0.1
    our_odom = Odometry()
    while not rospy.is_shutdown():
        if i == 0:
            our_odom.header.frame_id = 'map'
            our_odom.header.stamp = rospy.Time.now()
            our_odom.pose.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
            our_odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

        odometry_pub.publish(our_odom)

        i += 1
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
