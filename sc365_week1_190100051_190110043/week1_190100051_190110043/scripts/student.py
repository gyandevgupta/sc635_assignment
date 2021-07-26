#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def subscriber():
    sub_n = rospy.Subscriber('name_publish',String, callback1)
    sub_r = rospy.Subscriber('roll_publish',String, callback2)
    rospy.spin()
    

def callback1(message):
    global  x
    x = (message.data).split("_")
    
def callback2(message):
    global  y
    y = (message.data).split("_")
    rospy.loginfo("Student {} has roll :{} \nStudent {} has roll: {}".format(x[0],y[0],x[1],y[1]))

if __name__ == "__main__":
    rospy.init_node("student_data")
    subscriber()
