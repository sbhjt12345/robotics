#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
import turtlesim.srv
global_count = 0


def place_turtles():
    rospy.wait_for_service('spawn')
    print("in the show")
    try:
        spawner = rospy.ServiceProxy('spawn',turtlesim.srv.Spawn)
        jennie = spawner(2,2,0,'jennie')
        lisa = spawner(8,2,0,'lisa')
        jisoo = spawner(2,8,0,'jisoo')
        chaeyoung = spawner(8,8,0,'chaeyoung')
        rospy.init_node('turtle_square',anonymous = True)
        jennie_vel = rospy.Publisher('jennie/cmd_vel',Twist,queue_size=10)
        lisa_vel = rospy.Publisher('lisa/cmd_vel', Twist, queue_size=10)
        jisoo_vel = rospy.Publisher('jisoo/cmd_vel', Twist, queue_size=10)
        chaeyoung_vel = rospy.Publisher('chaeyoung/cmd_vel', Twist, queue_size=10)
        publishers = [jennie_vel,lisa_vel,jisoo_vel,chaeyoung_vel]
        count = 0
        while not rospy.is_shutdown() and count==0:
            draw_square(publishers)
            count = 1

    except rospy.ServiceException, e:
        print "Service call failed: %s" %e


def draw_square(publishers):
    jennie_msg = Twist()
    lisa_msg = Twist()
    jisoo_msg = Twist()
    chaeyoung_msg = Twist()
    list_msg = [jennie_msg, lisa_msg, jisoo_msg, chaeyoung_msg]
    for i in range(4):
        distance_count(publishers[i], list_msg[i], 0)
        angle_count(publishers[i], list_msg[i], 0)
        distance_count(publishers[i], list_msg[i], 0)
        angle_count(publishers[i], list_msg[i], 0)
        distance_count(publishers[i], list_msg[i], 0)
        angle_count(publishers[i], list_msg[i], 0)
        distance_count(publishers[i], list_msg[i], 0)
        angle_count(publishers[i], list_msg[i], 0)
        stop_moving(publishers[i], list_msg[i])


def distance_count(pub, vel_msg, current_distance):
    t0 = rospy.Time.now().to_sec()
    while current_distance < 2:
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
    while current_angle < math.pi/2:
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
        place_turtles()
    except rospy.ROSInterruptException: pass



