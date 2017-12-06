#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from foundations_hw4.srv import *
import numpy as np

myu = [np.zeros((2, 1)) for i in range(5)]
sigma = [np.identity(2) for i in range(5)]


def handle_p1(req):
    global myu
    global sigma
    rospy.wait_for_service('/vrep/youbot/position_sensor')
    try:
        get_pos = rospy.ServiceProxy('/vrep/youbot/position_sensor', GetPosition)
        A = np.identity(2)
        # B = np.zeros((2,2))
        C = np.identity(2)
        for i in range(5):
            #R = np.identity(2) * 0.1
            R = np.zeros((2,2))
            getp = get_pos(i + 1)
            Zt = np.array([[getp.loc.x], [getp.loc.y]])
            Qt = np.array([[getp.cov[0], getp.cov[1]], [getp.cov[2], getp.cov[3]]])
            # Rt = np.random.randn(2, 2) * 0.1
            new_sigma = A.dot(sigma[i]).dot(A.T) + R
            K = new_sigma.dot(C.T).dot(np.linalg.inv(C.dot(new_sigma).dot(C.T) + Qt))
            myu[i] = myu[i] + K.dot(Zt - C.dot(myu[i]))
            sigma[i] = (np.identity(2) - K.dot(C)).dot(new_sigma)
        print("OK so far")
        estimates = [Point(m[0][0], m[1][0], 0) for m in myu]
        confidence = [Point(s[0][0], s[1][1], 0) for s in sigma]
        return KalmanUpdateResponse(estimates, confidence)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def p1_server():
    rospy.init_node("p1", anonymous=True)
    print("hello")
    s = rospy.Service('p1', KalmanUpdate, handle_p1)
    rospy.spin()


if __name__ == '__main__':
    p1_server()