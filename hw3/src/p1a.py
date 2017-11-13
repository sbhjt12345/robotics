#!/usr/bin/env python

import rospy
from cvxopt import matrix, solvers
from geometry_msgs.msg import Point
from foundations_hw3.srv import GetPosition, SetPosition,PositionBucket

def handle_p1a(req):
    rospy.wait_for_service('/vrep/block1/pos/get')
    rospy.wait_for_service('/vrep/block2/pos/get')
    rospy.wait_for_service('/vrep/block3/pos/get')
    rospy.wait_for_service('/vrep/block4/pos/get')
    rospy.wait_for_service('/vrep/block5/pos/get')
    rospy.wait_for_service('/vrep/bucket/pos/set')
    try:
        x1_pos = rospy.ServiceProxy('/vrep/block1/pos/get', GetPosition)().pos
        x2_pos = rospy.ServiceProxy('/vrep/block2/pos/get', GetPosition)().pos
        x3_pos = rospy.ServiceProxy('/vrep/block3/pos/get', GetPosition)().pos
        x4_pos = rospy.ServiceProxy('/vrep/block4/pos/get', GetPosition)().pos
        x5_pos = rospy.ServiceProxy('/vrep/block5/pos/get', GetPosition)().pos
        x0_set = rospy.ServiceProxy('/vrep/bucket/pos/set', SetPosition)

        P = matrix([[10.0, 0.0], [0.0, 10.0]])
        sum_xi = (x1_pos.x + x2_pos.x + x3_pos.x + x4_pos.x + x5_pos.x) * (-2)
        sum_yi = (x1_pos.y + x2_pos.y + x3_pos.y + x4_pos.y + x5_pos.y) * (-2)
        q = matrix([sum_xi,sum_yi])
        sol = solvers.qp(P,q)
        x0_pos = Point(sol['x'][0], sol['x'][1], 0.0)
        print(x0_pos)
        x0_set(x0_pos)
        return []
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def p1a_server():
    rospy.init_node("p1a", anonymous=True)
    print("hello")
    s = rospy.Service('p1a', PositionBucket, handle_p1a)
    rospy.spin()

if __name__ == '__main__':
    p1a_server()









