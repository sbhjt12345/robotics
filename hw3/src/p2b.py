#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from foundations_hw3.srv import FollowPath, FollowPathResponse, InterpolatePath, ClosestPointPath
from tf.transformations import euler_from_quaternion
import numpy as np
from math import cos,sin,sqrt


def handle_p2b(req):
    global bot_pos
    rospy.wait_for_service('interpolate_path')
    rospy.wait_for_service('closest_point_path')
    rospy.wait_for_message('/vrep/youbot/base/pose', Pose)
    cmd_publisher = rospy.Publisher('/vrep/youbot/base/cmd_vel', Twist, queue_size=10)
    goal_path = req.path
    vel_msg = Twist()
    #point_bot_pos = bot_pos.position

    try:
        inte_path = rospy.ServiceProxy('interpolate_path', InterpolatePath)
        clop_path = rospy.ServiceProxy('closest_point_path', ClosestPointPath)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
        return FollowPathResponse()

    while distance(bot_pos.position, goal_path[-1]) > 0.57:
        (roll, pitch, yaw) = euler_from_quaternion([bot_pos.orientation.x, bot_pos.orientation.y,
                                                    bot_pos.orientation.z, bot_pos.orientation.w])
        tmp = clop_path(bot_pos.position, goal_path)
        path_position = tmp.path_position
        path_position.data += 0.7
        goal_p = inte_path(goal_path, path_position)  # point
        rotation_matrix = np.array([[cos(yaw), -sin(yaw), 0.0,0.0],
                                    [sin(yaw), cos(yaw), 0.0,0.0],
                                    [0.0, 0.0, 1.0,0.0],
                                    [0.0,0.0,0.0,1.0]]).T
        trans_matrix = np.array([[1.0,0.0,0.0,- bot_pos.position.x],
                                 [0.0,1.0,0.0,- bot_pos.position.y],
                                 [0.0,0.0,1.0,0.0],
                                 [0.0,0.0,0.0,1.0]])
        homo_matrix = np.dot(rotation_matrix,trans_matrix)
        global_goal_vector = np.array([goal_p.point.x, goal_p.point.y,0.0, 1.0]).T
        current_goal_vector = np.dot(homo_matrix, global_goal_vector)
        k = -2.0 * current_goal_vector[1] / (current_goal_vector[0] ** 2 + current_goal_vector[1] ** 2)
        print(yaw,k)
        vel_msg.linear.y = 1.0
        vel_msg.angular.z = k * 0.5
        cmd_publisher.publish(vel_msg)
        rospy.sleep(0.1)
    vel_msg.linear.y = 0.0
    vel_msg.angular.z = 0.0
    cmd_publisher.publish(vel_msg)

    return FollowPathResponse()


def distance(bot,block):
    return sqrt((bot.x - block.x)**2 + (bot.y - block.y)**2)

def callback_pos(data):
    global bot_pos
    bot_pos = data

def p2b_server():
    rospy.init_node("p2b", anonymous=True)
    print("hello")
    rospy.Subscriber('/vrep/youbot/base/pose', Pose, callback_pos)
    s = rospy.Service('p2b', FollowPath, handle_p2b)
    rospy.spin()


if __name__ == '__main__':
    p2b_server()