#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32,String


class p4c():
    def __init__(self):
        rospy.init_node("p4c",anonymous=True)
        self.rate_listener = rospy.Subscriber("/pub_rate",Int32,self.callback)
        self.num = Int32()
        self.ping_pub = rospy.Publisher("/ping",String,queue_size=10)

    def callback(self,data):
        self.num = data

    def print_msg(self):
        while self.num.data == 0:
            continue
        rate = rospy.Rate(self.num.data)
        self.ping_pub.publish("ping")
        print("its working")
        rospy.loginfo(self.num.data)
        rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    try:
        x = p4c()
        x.print_msg()
    except rospy.ROSInterruptException: pass


