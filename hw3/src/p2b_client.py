#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Point
from foundations_hw3.srv import FollowPath


def p2b_client():
    rospy.init_node("p2b_client",anonymous=True)
    rospy.wait_for_service('p2b')
    try:
        do = rospy.ServiceProxy('p2b', FollowPath)([Point(0.0,0.0,0.0),
                                                    Point(5.0,5.0,0.0),
                                                    Point(10.0,0.0,0.0)
                                                    ])
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == '__main__':
    p2b_client()