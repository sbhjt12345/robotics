#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math

class turtlebot():
    def __init__(self):
        rospy.init_node('henta_mover', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
        self.vel_msg = Twist()
        self.distance = 2
        self.angle = math.pi*(180-900/7)/180
        self.count = 0

    def heptagon_move(self):
        print("do some 7 angle moving")
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0
        while not rospy.is_shutdown():
            current_distance = 0
            self.distance_count(current_distance)
            current_angle = 0
            self.angle_count(current_angle)
            self.count += 1
            if self.count >= 7:
               self.vel_msg.linear.x = 0
               self.vel_msg.angular.z = 0
               self.velocity_publisher.publish(self.vel_msg)
               
               

    def distance_count(self,current_distance):
        t0 = rospy.Time.now().to_sec()
        while current_distance < self.distance and self.count < 7:
            self.vel_msg.linear.x = 1
            self.vel_msg.angular.z = 0
            self.velocity_publisher.publish(self.vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_distance = t1 - t0

    def angle_count(self,current_angle):
        t0 = rospy.Time.now().to_sec()
        while current_angle < self.angle and self.count < 7:
            self.vel_msg.linear.x = 0
            self.vel_msg.angular.z = math.pi / 6
            self.velocity_publisher.publish(self.vel_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = math.pi / 6 * (t1 - t0)

if __name__ == '__main__':
    try:
        x = turtlebot()
        x.heptagon_move()
    except rospy.ROSInterruptException: pass