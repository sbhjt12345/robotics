#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float64, Int64MultiArray, String
   

def callback(data):
    rospy.loginfo('I heard %s', data.data)
    
    
def listener(topic_name, topic_type):
    rospy.init_node('msg_listener',anonymous = True)
    if topic_type == 'Twist':
       rospy.Subscriber(topic_name,Twist,callback)
    elif topic_type == 'Point':
       rospy.Subscriber(topic_name,Point,callback)
    elif topic_type == 'Float64':
       rospy.Subscriber(topic_name,Float64,callback)
    elif topic_type == 'Int64MultiArray':
       rospy.Subscriber(topic_name,Int64MultiArray,callback)
    else:
       rospy.Subscriber(topic_name,String,callback)
    print(topic_type)
    rospy.spin()
    
if __name__ =='__main__':
    if len(sys.argv) != 3:
       print("you are not giving valid arguments")
    else:
       listener(sys.argv[1],sys.argv[2])



        
    
    
    
    
  

    
                      
    
    
    
    