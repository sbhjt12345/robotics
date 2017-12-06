#!/usr/bin/env python

import rospy
from foundations_hw4.srv import *
import numpy as np

beliefs = [0.2 for i in range(5)]

def handle_p2(req):
    global beliefs
    rospy.wait_for_service('/vrep/youbot/bomb_sensor')
    try:
        get_pos = rospy.ServiceProxy('/vrep/youbot/bomb_sensor', BombTest)
        for i in range(5):
            getp = get_pos(i+1)
            dis = getp.dist
            z = getp.is_bomb
            equ_5, equ_6 = equ_5_and_6(z, dis)
            equ_4 = equ_5 * beliefs[i] + equ_6 * (1-beliefs[i])
            beliefs[i] = (equ_5 * beliefs[i])/equ_4
        sum_b = np.sum(beliefs)
        new_beliefs = [b/sum_b for b in beliefs]
        return FindBombResponse(new_beliefs)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def equ_5_and_6(z, dis):
    equ_5 = (4 - np.tanh(3 * (dis - 1.5)) - np.tanh(3)) / 4
    equ_6 = 1 - equ_5
    if z:
        return equ_5, equ_6
    else:
        return equ_6, equ_5


def p2_server():
    rospy.init_node("p2", anonymous=True)
    print("hello")
    s = rospy.Service('p2', FindBomb, handle_p2)
    rospy.spin()


if __name__ == '__main__':
    p2_server()