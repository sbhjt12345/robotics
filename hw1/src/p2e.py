#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math

def s_move():
    rospy.init_node('stupid_mover',anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
    vel_msg = Twist()

    print("do some testttiing")
    vel_msg.linear.x = 2
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    while not rospy.is_shutdown():
        velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        s_move()
    except rospy.ROSInterruptException: pass