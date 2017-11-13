#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from foundations_hw3.srv import FindPath, ComputeCost, FindPathResponse


def cost_func(path):
    rospy.wait_for_service('compute_cost')
    try:
        cost_func = rospy.ServiceProxy('compute_cost', ComputeCost)
        return cost_func(path).cost
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def handle_p2a(req):
    start = req.start
    goal = req.goal
    n = req.num_waypoints
    delta_x = (start.x - goal.x) / (n - 1)
    delta_y = (start.y - goal.y) / (n - 1)
    way_points = [start]
    for i in range(1, n - 1):
        way_points.append(Point(start.x + delta_x * i,
                                start.y + delta_y * i,
                                0))
    way_points.append(goal)
    prev_cost = cost_func(way_points)
    new_path = []
    for i in range(500):
        new_path = gradient(way_points, n, 0.01)
        cur_cost = cost_func(new_path)
        rospy.loginfo("cur_cost is %f" % cur_cost)
        if abs(cur_cost - prev_cost) <= 0.00001:
            rospy.loginfo(new_path)
            return FindPathResponse(new_path)
        prev_cost = cur_cost
        way_points = new_path
    return FindPathResponse(new_path)



def gradient(path, n, delta):
    origin_cost = cost_func(path)
    finite_diff_x = []
    finite_diff_y = []
    finite_diff_x.append(path[0].x)
    finite_diff_y.append(path[0].y)
    for i in range(1, n-1):
        for j in range(2):
            copy_path = list(path)
            if j == 0:
                origin_x = path[i].x
                copy_path[i].x += delta
                cur_x = origin_x - 0.07 * (cost_func(copy_path) - origin_cost)/delta
                finite_diff_x.append(cur_x)
            else:
                origin_y = path[i].y
                copy_path[i].y += delta
                cur_y = origin_y - 0.07 * (cost_func(copy_path) - origin_cost) / delta
                finite_diff_y.append(cur_y)
    finite_diff_x.append(path[-1].x)
    finite_diff_y.append(path[-1].y)

    #print("len of x is %d" % len(finite_diff_x))   
    #print("n is %d" % n)

    new_path = []
    for i in range(n):
        new_path.append(Point(finite_diff_x[i],finite_diff_y[i],0.0))
    return new_path




def p2a_server():
    rospy.init_node("p2a", anonymous=True)
    print("hello")
    s = rospy.Service('p2a', FindPath, handle_p2a)
    rospy.spin()


if __name__ == '__main__':
    p2a_server()