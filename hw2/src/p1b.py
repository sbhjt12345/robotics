#!/usr/bin/env python

import rospy
from math import sin, cos, pi
import numpy as np
from geometry_msgs.msg import Point
from hw2.srv import P1b



def handle_p1b(req):
    print("here")
    theta = list(req.configs.angles)
    a = [0, 0.033, 0.155, 0.135]
    d = [0.1, 0, 0, 0]
    alpha = [0, pi/2, 0, 0]
    theta[0] += pi/2
    theta[1] += pi/2
    theta[3] += pi/2
    res_H = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    for i in range(4):
        R = np.array([[1, 0, 0, 0],
                     [0, cos(alpha[i]), -sin(alpha[i]), 0],
                     [0, sin(alpha[i]), cos(alpha[i]), 0],
                     [0, 0, 0, 1]])
        Q = np.array([[1, 0, 0, a[i]],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])
        P = np.array([[cos(theta[i]), -sin(theta[i]), 0, 0],
                     [sin(theta[i]), cos(theta[i]), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])
        final = np.array([[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, d[i]],
                         [0, 0, 0, 1]])
        cur_H = np.dot(np.dot(np.dot(R,Q),P),final)
        res_H = np.dot(res_H, cur_H)
    cur_point = np.array([0, -0.2175, 0, 1]).T
    global_point = np.dot(res_H, cur_point)
    pp = Point(global_point[0], global_point[1], global_point[2])
    pub.publish(pp)
    return pp


def p1b_server():
    rospy.init_node('p1b_server')
    global pub 
    pub = rospy.Publisher('/vrep/youbot/target/position', Point, queue_size=10)
    print("hello")
    s = rospy.Service('p1b', P1b, handle_p1b)
    print("i am ready")
    rospy.spin()


if __name__ == "__main__":
    p1b_server()