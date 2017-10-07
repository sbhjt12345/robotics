#!/usr/bin/env python
import rospy
from math import pi, cos, sin, sqrt
import numpy as np
from foundations_hw2.msg import JointAngles
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
from hw2.srv import P3a


def getpose(data):
    global pos
    pos = data.position


def getangle(data):
    global q
    q = data.angles


def jacobian_transpose(req):
    global pos
    global q
    end_point = req.goal_point
    tmpangles = [0, 0, 0, 0]
    tmpangles[0] = q[0]
    tmpangles[1] = q[1]
    tmpangles[2] = q[2]
    tmpangles[3] = q[3]
    e = Point(pos.x, pos.y, pos.z)
    count = 0
    while dis(e, end_point) >= 0.01 and count <= 3000:
        jaco_matrix = get_jacobian(tmpangles)
        de = 0.5 * np.array([[(end_point.x - e.x)],
                             [(end_point.y - e.y)],
                             [(end_point.z - e.z)]])
        dq = np.dot(jaco_matrix.T, de)
        for i in range(4):
            tmpangles[i] += dq[i]
        point_array = get_fk(tmpangles, 0, 0, 0, 0)
        e.x = point_array[0]
        e.y = point_array[1]
        e.z = point_array[2]
        count += 1
    rospy.loginfo(e)
    rospy.loginfo(end_point)

    res_config = JointAngles()
    res_config.angles = (tmpangles[0], tmpangles[1], tmpangles[2], tmpangles[3], 0)
    pub1.publish(tmpangles[0])
    pub2.publish(tmpangles[1])
    pub3.publish(tmpangles[2])
    pub4.publish(tmpangles[3])
    return res_config


def dis(PointA, PointB):
    tmp_dis = (PointA.x - PointB.x) ** 2 + (PointA.y - PointB.y) ** 2 + (PointA.z - PointB.z) ** 2
    dis = sqrt(tmp_dis)
    return dis


def get_fk(theta, x1, x2, x3, x4):
    alpha = [0, pi / 2, 0, 0]
    a = [0, 0.033, 0.155, 0.135]
    d = [0.1, 0, 0, 0]
    res_H = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    angles = [0, 0, 0, 0]
    angles[0] = theta[0] + pi / 2 + x1
    angles[1] = theta[1] + pi / 2 + x2
    angles[2] = theta[2] + x3
    angles[3] = theta[3] + pi / 2 + x4
    for i in range(4):
        R = np.array([[1, 0, 0, 0],
                      [0, cos(alpha[i]), -sin(alpha[i]), 0],
                      [0, sin(alpha[i]), cos(alpha[i]), 0],
                      [0, 0, 0, 1]])
        Q = np.array([[1, 0, 0, a[i]],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        P = np.array([[cos(angles[i]), -sin(angles[i]), 0, 0],
                      [sin(angles[i]), cos(angles[i]), 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        final = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, d[i]],
                          [0, 0, 0, 1]])
        cur_H = np.dot(np.dot(np.dot(R, Q), P), final)
        res_H = np.dot(res_H, cur_H)
    cur_point = np.array([0, -0.2175, 0, 1]).T
    global_point = np.dot(res_H, cur_point)
    return np.array([global_point[0], global_point[1], global_point[2]])


def get_jacobian(angles):
    delta = 0.0001
    f_original = get_fk(angles, 0, 0, 0, 0)
    f_theta1 = get_fk(angles, delta, 0, 0, 0)
    f_theta2 = get_fk(angles, 0, delta, 0, 0)
    f_theta3 = get_fk(angles, 0, 0, delta, 0)
    f_theta4 = get_fk(angles, 0, 0, 0, delta)
    J1 = (f_theta1 - f_original) / delta
    J2 = (f_theta2 - f_original) / delta
    J3 = (f_theta3 - f_original) / delta
    J4 = (f_theta4 - f_original) / delta
    return np.array([[J1[0], J2[0], J3[0], J4[0]],
                    [J1[1], J2[1], J3[1], J4[1]],
                    [J1[2], J2[2], J3[2], J4[2]]])


def IK_server():
    rospy.init_node('hw2_p3a', anonymous=True)

    rospy.wait_for_message('/vrep/youbot/arm/gripper/pose', Pose, timeout=None)
    rospy.Subscriber('/vrep/youbot/arm/gripper/pose', Pose, getpose)
    rospy.wait_for_message('/vrep/youbot/arm/pose', JointAngles, timeout=None)
    rospy.Subscriber('/vrep/youbot/arm/pose', JointAngles, getangle)

    global pub1,pub2,pub3,pub4

    pub1 = rospy.Publisher('/vrep/youbot/arm/joint1/angle', Float64, queue_size=10)
    pub2 = rospy.Publisher('/vrep/youbot/arm/joint2/angle', Float64, queue_size=10)
    pub3 = rospy.Publisher('/vrep/youbot/arm/joint3/angle', Float64, queue_size=10)
    pub4 = rospy.Publisher('/vrep/youbot/arm/joint4/angle', Float64, queue_size=10)

    server = rospy.Service('p3a', P3a, jacobian_transpose)

    rospy.spin()


if __name__ == '__main__':
    IK_server()