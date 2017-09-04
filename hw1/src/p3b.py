#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from turtlesim.msg import Pose
from foundations_hw1.srv import Escape
from math import sqrt, pow
import random


class p3b():
    def __init__(self):
        rospy.init_node("chaser_pose", anonymous=True)
        self.chaser_listener = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        self.chaser_pose = Pose()

    def running_server(self):
        s = rospy.Service('escape', Escape, self.run_turtle)
        rospy.spin()

    def callback(self, data):
        self.chaser_pose = data

    def run_turtle(self, req):
        while self.chaser_pose.x == 0 and self.chaser_pose.y == 0:
            continue
        goal = Point()
        x = random.random() * 12
        y = random.random() * 12
        while self.distance(x,y,self.chaser_pose.x,self.chaser_pose.y) < 0.5:
            x = random.random() * 12
            y = random.random() * 12
        goal.x = x
        goal.y = y
        return goal

    def distance(self, turtlex, turtley, targetx, targety):
        return sqrt(pow((turtlex - targetx), 2) + pow((turtley - targety), 2))



if __name__ == "__main__":
    runner = p3b()
    runner.running_server()