#!/usr/bin/env python

import rospy
import numpy as np 
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# from quat2euler import quat2euler, euler2quat

def callback (data):
    global pose
    # x  = data.pose.pose.orientation.x
    # y  = data.pose.pose.orientation.y
    # z = data.pose.pose.orientation.z
    # w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y]
def myodom():
    rospy.init_node('myOdometry')
    pub=rospy.Publisher('/odometry_tracker',Twist,queue_size=10)
    rospy.Subscriber('/odom', Odometry, callback)
    rate=rospy.Rate(5)
    velocity_msg =Twist()
    velocity_msg.linear.x = 0.083333
    velocity_msg.angular.z=0
    i=0
    global distance
    while not rospy.is_shutdown():
        if(i<10):
            rospy.sleep(2)
            pub.publish(velocity_msg)
            # print(i) # for debugging
            
        i=i+1
        distance = math.sqrt((pose[0]-0.5)**2 + (pose[1]-0)**2 )
        # pub.publish(velocity_msg)

if __name__ == '__main__':
    try:
        myodom()
        print('')
        # the distance of the bot from point (0.5,0)
        print('The distance is ', distance) 

    except rospy.ROSInterruptException:
        pass
