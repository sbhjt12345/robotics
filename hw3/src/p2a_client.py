#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Point
from foundations_hw3.srv import FindPath


def p2a_client():
    rospy.init_node("p2a_client",anonymous=True)
    rospy.wait_for_service('p2a')
    try:
        do = rospy.ServiceProxy('p2a', FindPath)(Point(0.0,0.0,0.0),Point(4.0,7.0,0.0),3)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == '__main__':
    p2a_client()