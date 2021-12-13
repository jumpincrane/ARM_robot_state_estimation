#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
import numpy as np

input_data = [0, 0, 0]
pos = [0, 0]


def callback_input(data):
    u1 = data.linear.x  # predkosc katowa pierwszego kola
    u2 = data.linear.y  # predkosc postepowa
    theta = data.angular.z  # theta skret

    global input_data
    input_data = [u1, u2, theta]


def calculations(u1, u2, theta, delta_t):
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
    # obliczanie pozycji i inkrementowanie jej o przemieszczenie
    x = q_d[1] * delta_t
    y = q_d[2] * delta_t
    global pos
    pos[0] = pos[0] + x
    pos[1] = pos[1] + y

    quat = quaternion_from_euler(0, 0, beta)
    quat = Quaternion(quat[0], quat[1], quat[2], quat[3])
    # trzeba przypisac wynik do zmiennej rosowej our_odom
    our_odom.header.frame_id = 'map'
    our_odom.header.stamp = rospy.Time.now()
    our_odom.pose.pose = Pose(Point(pos[0], pos[1], 0), quat)
    our_odom.twist.twist = Twist(Vector3(q_d[1], q_d[2], 0), Vector3(0, 0, q_d[0]))
    return our_odom


def main():
    odometry_pub = rospy.Publisher('Odometry_pub', Odometry, queue_size=10)
    rospy.Subscriber("/input_signal", Twist, callback_input)
    rospy.init_node('robot_node', anonymous=True)
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

        our_odom = calculations(input_data[0], input_data[1], input_data[2], delta_t)
        odometry_pub.publish(our_odom)

        i += 1
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
