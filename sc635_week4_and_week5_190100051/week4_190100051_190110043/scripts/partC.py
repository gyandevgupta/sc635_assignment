#!/usr/bin/env python

import rospy
import numpy as np 
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from quat2euler import quat2euler
pose = [0,0,0]
yaw = 0
def callback (data):
    global pose
    global yaw
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y,quat2euler(x,y,z,w)[2]]
def myodom():
    global pose
    rospy.init_node('myOdometry')
    pub=rospy.Publisher('/odometry_tracker',Twist,queue_size=10)
    rospy.Subscriber('/odom', Odometry, callback)
    rate=rospy.Rate(5)
    velocity_msg =Twist()
    i=0
    global distance
    while not rospy.is_shutdown():
        t1 = 2 
        t2 = 3
        for x2 in range(1,200):
            for y2 in range(1,200):
                i=0 
                mu = (1/2)*((pose[0]-x2)*math.cos(pose[2])-(pose[1]-y2)*math.sin(pose[2]))/((pose[1]-y2)*math.cos(pose[2])-(pose[0]-x2)*math.sin(pose[2]))
                xc = (pose[0]+x2)/2 + mu*(pose[1]-y2)
                yc = (pose[1]+y2)/2 + mu*(pose[0]-x2)
                rc = math.sqrt((pose[0]-x2)**2+(pose[1]-y2)**2)
                dtheta = math.atan2(y2 - yc,x2 -xc) - math.atan2(pose[1]-yc,pose[0]-xc)
                vhat = math.abs((dtheta/t1)*rc) 
                what = dtheta/t1
                yhat = (theta_prime - pose[2])/t1 - what 

                while(i<10):
                    velocity_msg.linear.x = vhat
                    velocity_msg.angular.z = what
                    rospy.sleep(2)
                    pub.publish(velocity_msg)
                    i=i+1

                mu = (1/2)*((pose[0]-x2)*math.cos(pose[2])-(pose[1]-y2)*math.sin(pose[2]))/((pose[1]-y2)*math.cos(pose[2])-(pose[0]-x2)*math.sin(pose[2]))
                xc = (pose[0]+x2)/2 + mu*(pose[1]-y2)
                yc = (pose[1]+y2)/2 + mu*(pose[0]-x2)
                rc = math.sqrt((pose[0]-x2)**2+(pose[1]-y2)**2)
                dtheta = math.atan2(y2 - yc,x2 -xc) - math.atan2(pose[1]-yc,pose[0]-xc)
                vhat = math.abs((dtheta/t1)*rc) 
                what = dtheta/t2
                yhat = (theta_prime - pose[2])/t2 - what

                while(i<10):
                    velocity_msg.linear.x = vhat
                    velocity_msg.angular.z = what
                    rospy.sleep(2)
                    pub.publish(velocity_msg)
                    i=i+1       

if __name__ == '__main__':
    try:
        myodom()
        print('')
        # the distance of the bot from point (0.5,0)
        print('The distance is ', distance) 

    except rospy.ROSInterruptException:
        pass
