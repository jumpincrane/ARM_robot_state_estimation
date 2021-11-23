#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import numpy as np
import time

input_data = [0, 0, 0]


def callback_input(data):
    u1 = data.linear.x
    u2 = data.linear.y
    theta = data.angular.z

    global input_data
    input_data = [u1, u2, theta]


def calculations(u1, u2, theta, delta_t, t):
    our_odom = Odometry()
    beta = u1 * delta_t
    l = 1
    v1 = u2 / l * np.tan(beta)
    v2 = u2
    matrixA = np.array([[1, 0],
                        [0, np.cos(theta)],
                        [0, np.sin(theta)]])
    v = np.array([v1, v2])

    q_d = matrixA @ v

    # trzeba przypisac wynik do zmiennej rosowej our_odom
    our_odom.header.frame_id = '/map'
    our_odom.header.stamp = t
    our_odom.pose.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
    our_odom.twist.twist = Twist(Vector3(q_d[0], q_d[1], 0), Vector3(0, 0, q_d[2]))
    return our_odom


def main():
    odometry_pub = rospy.Publisher('Odometry_pub', Odometry, queue_size=10)
    rospy.Subscriber("/input_signal", Twist, callback_input)
    rospy.init_node('robot_node', anonymous=True)
    rate = rospy.Rate(1)  # 100hz
    i = 0
    our_odom = Odometry()
    while not rospy.is_shutdown():

        if i == 0:
            t0 = time.time()
            t2 = 0
            our_odom.header.frame_id = '/map'
            our_odom.header.stamp = t0
            our_odom.pose.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
            our_odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

        t = time.time() - t0
        delta_t = t - t2
        our_odom = calculations(input_data[0], input_data[1], input_data[2], delta_t, t)
        odometry_pub.publish(our_odom)
        print(our_odom)

        i += 1
        rate.sleep()
        t2 = time.time() - t0


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
