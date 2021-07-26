#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publisher():

    pub = rospy.Publisher('roll_publish',String , queue_size=50)
    rate =rospy.Rate(1)

    msg_to_publish = String()


    while not rospy.is_shutdown():
        string_to_publish = "190110043_190100051"

        msg_to_publish.data = string_to_publish
        pub.publish(msg_to_publish)
        rate.sleep()

if __name__== "__main__":
    rospy.init_node("roll")
    publisher()
