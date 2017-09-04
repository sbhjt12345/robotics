#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import random

class Updater():
    def __init__(self):
        rospy.init_node('hz_mover', anonymous=True)
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
        self.vel_msg = Twist()
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0


    def iter_move(self):
         print("in the show")
         while not rospy.is_shutdown():
             rospy.Timer(rospy.Duration(1),self.callback)

    def callback(self,event):
        print("changing speed")
        speed = 1 + random.random() * 10
        self.vel_msg.linear.x = speed
        self.vel_msg.angular.z = speed
        self.vel_pub.publish(self.vel_msg)
        rospy.loginfo(self.vel_msg)

if __name__ == '__main__':
    try:
        x = Updater()
        x.iter_move()
    except rospy.ROSInterruptException: pass





