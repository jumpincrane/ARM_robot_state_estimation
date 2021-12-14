#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
import numpy as np

input_data = Odometry()


def callback_data(data):
    global input_data
    input_data = data


previous_q_c = np.array([0, 0, 0])
previous_f = np.array([0, 0, 0])
pos = [0, 0]


def predictor(delta_t):
    our_odom = Odometry()
    global previous_f
    global previous_q_c
    global pos

    q_p = previous_q_c + delta_t * previous_f

    x = input_data.twist.twist.linear.x * delta_t
    y = input_data.twist.twist.linear.y * delta_t
    theta = input_data.twist.twist.angular.y
    previous_f = np.array([x, y, theta])

    pos[0] = pos[0] + x
    pos[1] = pos[1] + y
    quat = quaternion_from_euler(0, 0, theta)
    quat = Quaternion(quat[0], quat[1], quat[2], quat[3])

    previous_q_c = q_p

    our_odom.header.frame_id = 'map'
    our_odom.header.stamp = rospy.Time.now()
    our_odom.pose.pose = Pose(Point(pos[0], pos[1], 0), quat)
    our_odom.twist.twist = Twist(Vector3(input_data.twist.twist.linear.x,
                                         input_data.twist.twist.linear.y, 0),
                                 Vector3(0, theta, input_data.twist.twist.angular.z))

    return our_odom


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

        our_odom = predictor(delta_t)
        odometry_pub.publish(our_odom)

        i += 1
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
