#!/usr/bin/env python


import rospy
import numpy as np
from math import sqrt
from hw2.srv import P1b, P2b, P3a, P3b
from foundations_hw2.msg import JointAngles
from geometry_msgs.msg import Pose, Point


def handle_p3b(req):
    #w1 = Point(1.5, 1.5, 1.5)
    #w2 = Point(1.5, 2.5, 1)
    #w3 = Point(1.5, 2.5, 1.5)
    #w4 = Point(2.5, 2.5, 0.5)
    #w5 = Point(3, 3, 1)
    w1 = Point(1.5,1.5,1.5)
    w2 = Point(1.5,1.5,0.5)
    w3 = Point(1.5,1.5,1)
    w4 = Point(2,2,1)
    w5 = Point(2,2,1.5)
    w6 = Point(2,2,0.5)                     #write 'H'
    rospy.wait_for_service('p3a')
    try:
        draw_w = rospy.ServiceProxy('p3a', P3a)
        draw_w(w1)
        draw_w(w2)
        draw_w(w3)
        draw_w(w4)
        draw_w(w5)
        draw_w(w6)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def p3b_server():
    rospy.init_node('p3b_server')
    print("hello")
    s = rospy.Service('p3b', P3b, handle_p3b)
    rospy.spin()
    
if __name__ == '__main__':
    p3b_server()
