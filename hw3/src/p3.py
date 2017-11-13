#!/usr/bin/env python

import rospy
from math import pi
import numpy as np
from geometry_msgs.msg import Point,Pose,Twist,Vector3
from std_msgs.msg import Float64
from foundations_hw3.srv import *
from std_srvs.srv import SetBool
from tf.transformations import euler_from_quaternion



def p3():
    # initialization:
    rospy.init_node("p3_client", anonymous=True)
    cmd_publisher = rospy.Publisher('/vrep/youbot/base/cmd_vel', Twist, queue_size=10)
    angle_pub = [rospy.Publisher('/vrep/youbot/arm/joint{}/angle'.format(i), Float64, queue_size=10)
                 for i in range(1,6)]

    # waiting_service
    rospy.wait_for_service('p1b')
    rospy.wait_for_service('p2a')
    rospy.wait_for_service('p2b')
    rospy.wait_for_service('/vrep/youbot/arm/reach')
    rospy.wait_for_service('/vrep/youbot/gripper/grip')
    rospy.wait_for_service('/vrep/bucket/pos/get')
    for i in range(1,6):
        rospy.wait_for_service('/vrep/block{}/pos/get'.format(i))

    # main part
    try:
        # serviceproxy
        rospy.ServiceProxy('p1b', PositionBucket)()
        find_path = rospy.ServiceProxy('p2a', FindPath)
        follow_path = rospy.ServiceProxy('p2b',FollowPath)
        reach_block = rospy.ServiceProxy('/vrep/youbot/arm/reach',ReachPos)
        bucket_pos = rospy.ServiceProxy('/vrep/bucket/pos/get', GetPosition)().pos
        bucket_pos.z += 0.3
        grip_block = rospy.ServiceProxy('/vrep/youbot/gripper/grip',SetBool)
        position_clients = [
            rospy.ServiceProxy('/vrep/block{}/pos/get'.format(i), GetPosition) for i in range(1, 6)
        ]

        # iteration
        for i in range(1,6):
            #bucket_pos = rospy.ServiceProxy('/vrep/bucket/pos/get', GetPosition)().pos
            #bucket_pos.z += 0.3
            bot_pos = rospy.wait_for_message('/vrep/youbot/base/pose', Pose)
            start_point = bot_pos.position
            goal_point = position_clients[i]()
            g_pos,g_ori = goal_point.pos, goal_point.orientation
            path = find_path(start_point, g_pos, 3).path
            follow_path(path)
            (x_roll, x_pitch, x_yaw) = euler_from_quaternion([g_ori.x, g_ori.y,
                                                              g_ori.z, g_ori.w])
            while x_yaw > pi / 2:
                x_yaw -= pi / 2
            while x_yaw < -pi / 2:
                x_yaw += pi / 2

            while True:
                start_ori = rospy.wait_for_message('/vrep/youbot/base/pose', Pose).orientation
                (roll, pitch, yaw) = euler_from_quaternion([start_ori.x, start_ori.y,
                                                            start_ori.z, start_ori.w])
                while yaw > pi/2:
                    yaw -= pi/2
                while yaw < -pi/2:
                    yaw += pi/2

                cmd_publisher.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, -yaw)))
                print("NO! it's not parallel!!")
                if np.isclose(yaw, x_yaw, atol=0.09):
                    cmd_publisher.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
                    print("OK MAMA")
                    break
                rospy.sleep(0.01)

            print("we are moved!")
            reach_block(g_pos)
            rospy.sleep(2)
            grip_block(True)
            rospy.sleep(5)

            for i in range(5):
                angle_pub[i].publish(0.0)
            rospy.sleep(2)
            
            bot_pos = rospy.wait_for_message('/vrep/youbot/base/pose', Pose)
            start_point = bot_pos.position
            path = find_path(start_point, bucket_pos, 3).path
            follow_path(path)
            reach_block(bucket_pos)
            rospy.sleep(2)
            grip_block(False)
            rospy.sleep(5)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == '__main__':
    p3()




