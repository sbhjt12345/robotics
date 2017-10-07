#!/usr/bin/env python

import rospy
from foundations_hw2.srv import Interpolate
from foundations_hw2.msg import EulerAngles



def euler_angle():
    rospy.wait_for_service('foundations_hw2/interpolate_problem')
    try:
        rospy.init_node("hw2p4a",anonymous=True)
        haha = rospy.Publisher("vrep/shape_pose", EulerAngles, queue_size=10)
        rate = rospy.Rate(10)
        get_EA = rospy.ServiceProxy('foundations_hw2/interpolate_problem', Interpolate)()

        EA_initial = get_EA.initial
        EA_final = get_EA.final
        given_second = get_EA.seconds
        delta_phi = (EA_final.phi - EA_initial.phi) / (given_second * 10)
        delta_theta = (EA_final.theta - EA_initial.theta) / (given_second * 10)
        delta_psi = (EA_final.psi - EA_initial.psi) / (given_second * 10)
        EA = EulerAngles()
        print("in the show")

        for i in range(int(given_second) * 10):
            EA.phi = EA_initial.phi + delta_phi * i
            EA.theta = EA_initial.theta + delta_theta * i
            EA.psi = EA_initial.psi + delta_psi * i
            haha.publish(EA)
            rospy.loginfo(EA)
            rate.sleep()
        rospy.spin()

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        
if __name__ == '__main__':
    euler_angle()




