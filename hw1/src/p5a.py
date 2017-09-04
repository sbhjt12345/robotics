#!/usr/bin/env python

import rospy
import sys
import math
from geometry_msgs.msg import Twist
import turtlesim.srv

def place_turtles(xPos, yPos, name):
    rospy.wait_for_service('spawn')
    print("in the show")
    try:
        spawner = rospy.ServiceProxy('spawn',turtlesim.srv.Spawn)
        single_turtle = spawner(xPos, yPos, 0.0, name+'')
        name_topic = '/'+name+'/cmd_vel'
        rospy.init_node("p5a",anonymous=True)
        st_pub = rospy.Publisher(name_topic, Twist, queue_size=10)
        count = 0
        while not rospy.is_shutdown() and count==0:
            draw_square(st_pub)
            count = 1
    except rospy.ServiceException, e:
        print "Service call failed: %s" %e

def draw_square(pub):
    turtle_msg = Twist()
    distance_count(pub,turtle_msg,0)
    angle_count(pub,turtle_msg,0)
    distance_count(pub, turtle_msg, 0)
    angle_count(pub, turtle_msg, 0)
    distance_count(pub, turtle_msg, 0)
    angle_count(pub, turtle_msg, 0)
    distance_count(pub, turtle_msg, 0)
    angle_count(pub, turtle_msg, 0)
    stop_moving(pub,turtle_msg)


def distance_count(pub,vel_msg,current_distance):
    t0 = rospy.Time.now().to_sec()
    while current_distance < 1:
        vel_msg.linear.x = 1
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        pub.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = t1 - t0


def angle_count(pub, vel_msg, current_angle):
    t0 = rospy.Time.now().to_sec()
    while current_angle < math.pi / 2:
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = math.pi/4
        pub.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = math.pi/4 * (t1 - t0)


def stop_moving(pub, vel_msg):
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    print("reached here")
    pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        #if len(sys.argv) != 4:
        #    print("wrong input")
        #else:
        #    print(type(sys.argv[1]))
        #    print(type(sys.argv[2]))
        #    print(type(sys.argv[3]))
    #        place_turtles(float(sys.argv[1]),float(sys.argv[2]),sys.argv[3])
        #namespace = rospy.get_namespace()
        #print(namespace)
        #xPos = rospy.get_param('p5a/xPos')
        #yPos = rospy.get_param('p5a/yPos')
        #turname = rospy.get_param('p5a/turname')
        xPos = float(sys.argv[1])
        yPos = float(sys.argv[2])
        turname = sys.argv[3]
        place_turtles(xPos,yPos,turname)
    except rospy.ROSInterruptException: pass




