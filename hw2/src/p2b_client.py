#!/usr/bin/env python

import rospy
from foundations_hw2.msg import JointAngles
from hw2.srv import P2b



def callback(data):
    global ja
    ja = data



def test_p2b():
    global ja
    rospy.init_node("hahahahaha")
    rospy.Subscriber('foundations_hw2/arm_config', JointAngles, callback)
    rospy.wait_for_service('p2b')
    try:
        while not rospy.is_shutdown():
            rospy.wait_for_message('foundations_hw2/arm_config', JointAngles, timeout=None)
            print(ja.angles)
            s = rospy.ServiceProxy('p2b', P2b)
            jaco = s(ja).ja_matrix.data
            print(jaco)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e



if __name__ == '__main__':
    test_p2b()
