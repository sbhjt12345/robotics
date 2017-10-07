#!/usr/bin/env python

import rospy
from foundations_hw2.srv import Interpolate
from foundations_hw2.msg import EulerAngles
from math import sqrt, sin, cos, acos, atan2, asin, fabs
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np


def slerp(v0, v1, t):
    dot = v0.dot(v1)
    if fabs(dot) > 0.9995:
        return v0 + t * (v1 - v0)
    if dot < 0:
        v1 = -v1
        dot = -dot
    half_theta = acos(dot) * t
    sin_half_theta = sqrt(1 - dot ** 2)
    ratio_A = sin((1 - t) * half_theta) / sin_half_theta
    ratio_B = sin(t * half_theta) / sin_half_theta
    return v0 * ratio_A + v1 * ratio_B


def quaternion():
    rospy.wait_for_service('foundations_hw2/interpolate_problem')
    try:
        rospy.init_node("hw2p4b", anonymous=True)
        haha = rospy.Publisher("vrep/shape_pose", EulerAngles, queue_size=10)
        rate = rospy.Rate(10)
        get_EA = rospy.ServiceProxy('foundations_hw2/interpolate_problem', Interpolate)()

        EA_initial = get_EA.initial    #phi for z, theta for y, psi for x
        EA_final = get_EA.final
        ini_quaternion = quaternion_from_euler(EA_initial.psi, EA_initial.theta, EA_initial.phi)
        fin_quaternion = quaternion_from_euler(EA_final.psi, EA_final.theta, EA_final.phi)

        given_second = int(get_EA.seconds)
        t = 0.0
        print("in the show")
        for i in range(given_second * 10):
            t += 1.0/(given_second*10)
            qm = slerp(ini_quaternion, fin_quaternion, t)
            euler = euler_from_quaternion(qm)

            tmp_EA = EulerAngles()
            tmp_EA.psi = euler[0]
            tmp_EA.theta = euler[1]
            tmp_EA.phi = euler[2]
            haha.publish(tmp_EA)
            rospy.loginfo(tmp_EA)
            rate.sleep()

        rospy.spin()

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e




if __name__ == '__main__':
    quaternion()

