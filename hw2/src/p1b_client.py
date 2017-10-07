#!/usr/bin/env python

import rospy
from math import sin, cos, pi
import numpy as np
from geometry_msgs.msg import Point
from foundations_hw2.msg import JointAngles
from hw2.srv import P1b
from std_msgs.msg import Float64


def callback(data):
    global ja
    ja = data
    


def test_p1b():
    global ja
    rospy.init_node("hahahahaha")
    #pub = rospy.Publisher('/vrep/youbot/target/position', Point, queue_size=10)
    pub1 = rospy.Publisher('/vrep/youbot/arm/joint1/angle', Float64, queue_size=10)
    pub2 = rospy.Publisher('/vrep/youbot/arm/joint2/angle', Float64, queue_size=10)
    pub3 = rospy.Publisher('/vrep/youbot/arm/joint3/angle', Float64, queue_size=10)
    pub4 = rospy.Publisher('/vrep/youbot/arm/joint4/angle', Float64, queue_size=10)
    pub5 = rospy.Publisher('/vrep/youbot/arm/joint5/angle', Float64, queue_size=10)
    rospy.Subscriber('foundations_hw2/arm_config', JointAngles, callback)
    rospy.wait_for_service('p1b')
    try:
        while not rospy.is_shutdown():
            rospy.wait_for_message('foundations_hw2/arm_config', JointAngles, timeout=None)
            print(ja.angles)
            s = rospy.ServiceProxy('p1b', P1b)
            po = s(ja).point
            print("it is %f, %f, %f" % (po.x, po.y, po.z))

            #pub.publish(po)
            pub1.publish(ja.angles[0])
            pub2.publish(ja.angles[1])
            pub3.publish(ja.angles[2])
            pub4.publish(ja.angles[3])
            pub5.publish(ja.angles[4])
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        


if __name__ == '__main__':
    test_p1b()



