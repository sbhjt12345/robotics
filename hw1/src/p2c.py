#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from math import pow,atan2,sqrt


#idea: 
class turtleTracker():

    def __init__(self):
        rospy.init_node('turtle_tracker',anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10)
        self.point_listener = rospy.Subscriber('/hw1/target_loc',Point,self.callback)
        self.point = Point()
        self.turtlepos_listener = rospy.Subscriber('/turtle1/pose',Pose,self.callbackTurtle)
        self.pose = Pose()
        #self.rate = rospy.Rate(10)
        
    def callback(self,data):
        self.point = data
        self.point.x = round(self.point.x,4)
        self.point.y = round(self.point.y,4)
        #self.point.z = round(self.point.z,4)
        
    def callbackTurtle(self,data):
        self.pose = data
        self.pose.x = round(self.pose.x,4)
        self.pose.y = round(self.pose.y,4)
        
    def distance(self,turtlex,turtley,targetx,targety):
        #tthe ddistance between target turtlle aand turtle1
        return sqrt(pow((turtlex - targetx),2) + pow((turtley - targety),2))
        
    
    def moveToTarget(self):
        print("we are in the show")
        vel_msg = Twist()
        while True:
        
            while self.distance(self.pose.x,self.pose.y,self.point.x,self.point.y) > 0.05:
              #according to piazza only linear x and anguularr z matter
                vel_msg.linear.x = 1.2 * self.distance(self.pose.x,self.pose.y,self.point.x,self.point.y)
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 1.2 * (atan2(self.point.y-self.pose.y,self.point.x-self.pose.x)-self.pose.theta)
                self.velocity_publisher.publish(vel_msg)
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
        rospy.spin()
        
        
if __name__ == '__main__':
    try:
        tt = turtleTracker()
        while tt.pose.x==0 and tt.pose.y==0:
            continue
        while tt.point.x==0 and tt.point.y==0:
            continue
        #print("the point is %f and %f"%(tt.point.x,tt.point.y))               
        tt.moveToTarget()
        
    except rospy.ROSInterruptException: pass  
        
        
        
        
        


    
                      
    
    
    
    