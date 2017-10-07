#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from foundations_hw2.msg import Ackermann
from math import atan2, sqrt

cmd_vel = Twist()


def p5():
    rospy.init_node("hw2_p5", anonymous=True)
    ackermann_publisher = rospy.Publisher('vrep/ackermann/cmd_vel', Ackermann, queue_size=10)
    youbot_publisher = rospy.Publisher('vrep/youbot/base/cmd_vel', Twist, queue_size=10)
    #rate = rospy.rate(10)

    def callback(data):
        global cmd_vel
        wheelbase = 2.5772
        distance = 1.5103
        while data.linear.x == 0 and data.linear.y == 0 and data.angular.z == 0:
            continue
        cmd_vel = data
        velocity = sqrt(cmd_vel.linear.x ** 2 + cmd_vel.linear.y ** 2)
        radius = velocity / cmd_vel.angular.z
        steering_angle = atan2(wheelbase, radius)
        ack = Ackermann(steering_angle, velocity)
        ackermann_publisher.publish(ack)
        youbot_publisher.publish(cmd_vel)

    rospy.Subscriber('foundations_hw2/cmd_vel', Twist, callback)
    rospy.spin()


if __name__ == '__main__':
    p5()
