#!/usr/bin/env python
import rospy
import numpy as np
import math
import time
from geometry_msgs.msg import Twist
from week3.msg import Trilateration

Landmarks = []
pose_z = 0.00
Kp = 0.50
Kp_w = 0.18
global pose_x
global pose_y
pose_x = 0
pose_y = 0
def waypoint(t):
    x = 5*math.cos(t*10*np.pi/180)
    y = 5*math.sin(t*10*np.pi/180)
    return [x,y]
    
def callback(data):
    global pose_x
    global pose_y
    x1 = data.landmarkA.x
    y1 = data.landmarkA.y
    x2 = data.landmarkB.x
    y2 = data.landmarkB.y
    x3 = data.landmarkC.x
    y3 = data.landmarkA.y
    r1 = data.landmarkA.distance
    r2 = data.landmarkB.distance
    r3 = data.landmarkC.distance
    pose_y = (((r1**2-r2**2+x2**2-x1**2+y2**2-y1**2)/(x2-x1))- ((r1**2-r3**2+x3**2-x1**2+y3**2-y1**2)/(x3-x1)))/2*(((y2-y1)/(x2-x1))-((y3-y1)/(x3-x1)))
    pose_x = (((r1**2-r2**2+y2**2-y1**2+x2**2-x1**2)/(y2-y1))- ((r1**2-r3**2+y3**2-y1**2+x3**2-x1**2)/(y3-y1)))/2*(((x2-x1)/(y2-y1))-((x3-x1)/(y3-y1)))
    print ("x=" + pose_x + " " + "y="+pose_y)
    
def angle(theta):
    if(theta > np.pi):
        theta = -(2*np.pi-theta)
    elif(theta < -np.pi):
        theta = (2*np.pi+theta)
    return theta

def control():
    global pose_x
    global pose_y
    global pose_z
    t = 0
    rospy.init_node('kobuki')
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    rospy.Subscriber('/trilateration_data', Trilateration, callback)
    rate = rospy.Rate(5) 
    
    d_err = 2.0
    rot   = False
    while not rospy.is_shutdown():
        [goalx, goaly]  = waypoint(t)
        err           = ((goalx-pose_x)**2 + (goaly-pose_y**2))**0.5
        theta_ref       = math.atan2((goaly-pose_y),(goalx-pose_x))
        theta_err       = angle(theta_ref - pose_z)
        velocity_msg    = Twist()

        
        if(abs(theta_err) > (2.0*np.pi/180)):
            if rot:
                velocity_msg.angular.z = -Kp_w*theta_err
            else:
                velocity_msg.linear.x  = 0.15
            rot = not rot
        else:
            velocity_msg.linear.x  = Kp*err

        if d_err < 0.3:
            velocity_msg = Twist()
            t = t + 1
        
        pub.publish(velocity_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass
