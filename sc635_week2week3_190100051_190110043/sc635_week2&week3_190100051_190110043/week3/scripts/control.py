#!/usr/bin/env python
import rospy
import numpy as np
from quat2euler import quat2euler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
import math

pose_x = 0.0
pose_y = 0.0
pose_t = 0.0
a = 1
b = 1
A = 4
totalPoints = 200
t = []
X = []
Y = []

def callback(data):
    global pose_x
    global pose_y
    global pose_t
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose_x = data.pose.pose.position.x
    pose_y = data.pose.pose.position.y
    pose_t = quat2euler(x,y,z,w)[2]
    
def waypoint():
    global t
    global X
    global Y
    t = []
    X = []
    Y = []
    for i in range(totalPoints+10):
        t.append(i*(2.0*math.pi)/(totalPoints))
    for i in range(totalPoints+10):
        X.append(A*math.cos(a*t[i]))
        Y.append(A*math.sin(b*t[i]))

def E_theta(point):
    dif = math.atan((point[1]-pose_y)/(point[0]-pose_x))
    return dif

def dist(point1,point2):
    return sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)

def angle(theta):
    if(theta > np.pi):
        theta = -(2*np.pi-theta)
    elif(theta < -np.pi):
        theta = (2*np.pi+theta)
    return theta

def control_loop():
    goal = Point()
    waypoint()
    global X
    global Y
    global pose_x
    global pose_y
    global pose_t

    rospy.init_node('kobuki')
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=5)
    rospy.Subscriber('/odom', Odometry, callback)
    i = 0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        velocity_msg = Twist()
        goal.x  = X[i]
        goal.y  = Y[i]
        theta   = E_theta((goal.x,goal.y))
        delta_t   = angle(theta  - pose_t)
        K_linear = 0.4
        K_angular = 0.8
        error   = dist((pose_x,pose_y),(goal.x,goal.y))
        if(abs(delta_t) > 0.22):
            velocity_msg.angular.z = K_angular*delta_t
        else:
            velocity_msg.linear.x = K_linear*error
    
        if(error < 0.25):
	    i = i + 1
        if(i == totalPoints):
            i = 0
        pub.publish(velocity_msg)

        print(str(goal.x)+" "+str(goal.y))
        rate.sleep()

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass