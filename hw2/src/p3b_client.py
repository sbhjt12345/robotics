#!/usr/bin/env python


import rospy
from hw2.srv import P1b, P2b, P3a, P3b

def p3b_client():
    rospy.init_node("p3b_client",anonymous=True)
    rospy.wait_for_service('p3b')
    try:
        rospy.ServiceProxy('p3b', P3b)()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == '__main__':
    p3b_client()
