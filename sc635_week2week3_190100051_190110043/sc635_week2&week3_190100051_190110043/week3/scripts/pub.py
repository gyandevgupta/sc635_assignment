#!/usr/bin/env python
import rospy
from week3.msg import Student_info

def talker():
    rospy.init_node('talker_node', anonymous=True)
    pub = rospy.Publisher('talker_topic', Student_info, queue_size=10)
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        t = Student_info("Vivek", 164230001)
        pub.publish(t)
        rospy.loginfo("Sent a message!")
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
