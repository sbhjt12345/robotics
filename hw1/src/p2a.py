#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist


def move():
    rospy.init_node('simple_mover',anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
    vel_msg = Twist()
    
    
    print("tuuurtlle is moviiiiing")
    vel_msg.linear.x = 2
    vel_msg.linear.y = 2
    vel_msg.linear.z = 2
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 1
    i = 0
    while not rospy.is_shutdown():
        while i < 200000:
            velocity_publisher.publish(vel_msg)
            i += 1
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        velocity_publisher.publish(vel_msg)
    
                        
               
if  __name__ == '__main__':
       try:
           move()
       except rospy.ROSInterruptException: pass     
                
    
    
    
    
    