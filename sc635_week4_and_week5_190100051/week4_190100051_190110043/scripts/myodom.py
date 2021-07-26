#!/usr/bin/env python
import rospy
import numpy as np
import math
import time
from geometry_msgs.msg import Twist

# def callback(data):
# 	rospy.loginfo("posex: " + data.x + "posey: " + data.y + "theta: " + data.theta)

def myodom():
	rospy.init_node('myodomtry')
	pub = rospy.Publisher('/odometry_tracker', Twist, queue_size=10)
	rate = rospy.Rate(5)
	velocity_msg = Twist()
	velocity_msg.linear.x = 0.11
	velocity_msg.angular.z = 0.8
	while not rospy.is_shutdown():
		#rospy.Subscriber('/manual_odom', Twist, callback)
		#rospy.spin()
		pub.publish(velocity_msg)
		rate.sleep()


if __name__ == '__main__':
    try:
        myodom()
    except rospy.ROSInterruptException:
        pass
