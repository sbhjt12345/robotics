#!/usr/bin/env python

import rospy
from foundations_hw2.msg import JointAngles
from hw2.srv import P3a
from geometry_msgs.msg import Point
from std_msgs.msg import Float64


def callback(data):
    global end_p
    end_p = data

def test_p3a():
    print("start")
    global end_p
    rospy.init_node("p3atest",anonymous=True)
    #pub = rospy.Publisher('/vrep/youbot/target/position', Point, queue_size=10)
    #pub1 = rospy.Publisher('/vrep/youbot/arm/joint1/angle', Float64, queue_size=10)
    #pub2 = rospy.Publisher('/vrep/youbot/arm/joint2/angle', Float64, queue_size=10)
    #pub3 = rospy.Publisher('/vrep/youbot/arm/joint3/angle', Float64, queue_size=10)
    #pub4 = rospy.Publisher('/vrep/youbot/arm/joint4/angle', Float64, queue_size=10)
    #pub5 = rospy.Publisher('/vrep/youbot/arm/joint5/angle', Float64, queue_size=10)
    rospy.Subscriber('foundations_hw2/effector_position',Point,callback)
    rospy.wait_for_service('p3a')
    print("sb")
    try: 
        while not rospy.is_shutdown():
            rospy.wait_for_message('foundations_hw2/effector_position', Point, timeout=None)
            print("end point is %f, %f, %f" % (end_p.x, end_p.y, end_p.z))

            s = rospy.ServiceProxy('p3a', P3a)
            configs = s(end_p).goal_configs
            print(configs.angles)
            ja = list(configs.angles)
            #pub1.publish(ja[0])
            #pub2.publish(ja[1])
            #pub3.publish(ja[2])
            #pub4.publish(ja[3])
            #pub5.publish(ja[4])
            print("it is working")
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == '__main__':
    print("ssssb")
    test_p3a()