#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from tf.transsformations import euler_from_quaternioin
from nav_msgs.msg import Odometry
import numpy as np
import time


def callback_input(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global input_data
    u1 = data.linear.x
    u2 = data.linear.y

    # Actualization of the Odometry message to bo published
    x = data.odometry.pose.position.x
    y = data.odometry.pose.position.y
    beta = data.odometry.pose.position.z
    quat = data.odometry.pose.pose.rotation
    theta = euler_from_quaternioin([quat.x, quat.y, quat.z, quat.w])[2]

    # theta = data.angular.z
    input_data = [u1, u2, theta]
    


def calculations(u1, u2, theta, t):
    our_odom = Odometry()
    u1, u2, theta = input_data
    beta = u1 * t
    l = 1
    v1 = u2/l * np.tan(beta)
    v2 = u2
    matrixA = np.array([[1, 0],
                        [0, np.cos(theta)],
                        [0, np.sin(theta)]])
    v = np.array([v1, v2])
    
    q_d = matrixA @ v
    
    # trzeba przypisac wynik do zmiennej rosowej our_odom
    q_d[0]
    q_d[1]
    q_d[2]

    return our_odom


def main():
    odometry_pub = rospy.Publisher('Odometry_pub', Odometry, queue_size=10)
    rospy.Subscriber("sub_input_data", Twist, callback_input)
    rospy.init_node('robot_node', anonymous=True)
    rate = rospy.Rate(1)  # 100hz

    while not rospy.is_shutdown():
        if i == 0:
            t0 = time.time()
        
        t = time.time() - t0
        delta_t = t - t2
        
        our_odom = calculations(u1, u2, delta_t)
        our_odom.header.timestamp = t
        
        odometry_pub.publish(our_odom)
        i += 1
        rate.sleep()
        t2 = time.time() - t0


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
