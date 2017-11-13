#!/usr/bin/env python


import rospy
from foundations_hw3.srv import PositionBucket

def p1a_client():
    rospy.init_node("p1a_client",anonymous=True)
    rospy.wait_for_service('p1a')
    try:
        do = rospy.ServiceProxy('p1a', PositionBucket)()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == '__main__':
    p1a_client()