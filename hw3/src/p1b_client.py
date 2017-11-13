#!/usr/bin/env python


import rospy
from foundations_hw3.srv import PositionBucket


def p1b_client():
    rospy.init_node("p1b_client",anonymous=True)
    rospy.wait_for_service('p1b')
    try:
        do = rospy.ServiceProxy('p1b', PositionBucket)()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == '__main__':
    p1b_client()