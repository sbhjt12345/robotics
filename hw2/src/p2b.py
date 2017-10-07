#!/usr/bin/env python

import rospy
from math import sin, cos, pi
import numpy as np
from geometry_msgs.msg import Point
from hw2.srv import P2b
from std_msgs.msg import Float64MultiArray, MultiArrayDimension


def computeH(x1, x2, x3, x4):
    global theta
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


def handle_p2b(req):
    global theta
    print("here")
    theta = list(req.configs.angles)
    delta = 0.0001
    f_original = computeH(0, 0, 0, 0)
    f_theta1 = computeH(delta, 0, 0, 0)
    f_theta2 = computeH(0, delta, 0, 0)
    f_theta3 = computeH(0, 0, delta, 0)
    f_theta4 = computeH(0, 0, 0, delta)
    J1 = (f_theta1 - f_original) / delta
    J2 = (f_theta2 - f_original) / delta
    J3 = (f_theta3 - f_original) / delta
    J4 = (f_theta4 - f_original) / delta

    jacobian = Float64MultiArray()
    jacobian.layout.dim = [MultiArrayDimension('height', 4, 3 * 4),
                           MultiArrayDimension('width', 3, 3)]
    jacobian.data = [J1[0], J2[0], J3[0], J4[0],
                     J1[1], J2[1], J3[1], J4[1],
                     J1[2], J2[2], J3[2], J4[2]]
    return jacobian


def p2b_server():
    rospy.init_node('p2b_server')
    print("hello")
    s = rospy.Service('p2b', P2b, handle_p2b)
    print("i am ready")
    rospy.spin()


if __name__ == "__main__":
    p2b_server()