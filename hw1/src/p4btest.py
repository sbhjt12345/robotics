#!/usr/bin/env python


import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, UInt32MultiArray, MultiArrayLayout, MultiArrayDimension


class LeetCode():
    def __init__(self):
        rospy.init_node("leetcode_turtle", anonymous=True)
        self.array_listener = rospy.Subscriber('/turtle1/image_sensor', Float64MultiArray, self.callback)
        self.listener = Float64MultiArray()
        self.flatten_array = []
        self.rate = rospy.Rate(20)
        self.maxsub_pub = rospy.Publisher('/hw1/subarray', UInt32MultiArray, queue_size=20)

    def callback(self,data):
        self.listener = data

    def flatten(self):
        #tran_array = np.split(np.array(self.listener.data),240)
        for i in self.listener.image_data:
            tempa,tempb = self.max_array(i)
            while tempa <= tempb:
                self.flatten_array.append(i[tempa])
                tempa += 1
        finala,finalb = self.max_array(self.flatten_array)
        layout = MultiArrayLayout()
        layout.dim = [MultiArrayDimension('height', 1, 2*1), MultiArrayDimension('width', 2, 2)]
        index_data = [finala, finalb]
        self.maxsub_pub.publish(layout,index_data)
        print("in the show")
        rospy.loginfo("calculating the work")
        print(index_data)
        rospy.spin()

    def max_array(self,nums):
        maxSum = float('-inf')
        beginIndex = 0
        endIndex = 0
        sumEndingHere = 0
        temp = 0
        for idx,val in enumerate(nums):
            sumEndingHere += val
            if sumEndingHere > maxSum:
                maxSum = sumEndingHere
                endIndex = idx
                beginIndex = temp
            elif sumEndingHere <= 0:
                sumEndingHere = 0
                temp = idx + 1

        return beginIndex, endIndex
        
if __name__ == '__main__':
    try:
        x = LeetCode()
        while len(x.listener.data)==0:
            continue
        x.flatten()
    except rospy.ROSInterruptException: pass



