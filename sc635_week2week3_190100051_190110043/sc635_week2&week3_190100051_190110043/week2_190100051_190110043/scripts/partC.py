#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist
import math
import time
from nav_msgs.msg import Odometry
from quat2euler import quat2euler
from matplotlib import pyplot as plt
x=0
y=0
yaw=0 
n = 100
t = np.linspace(0, 2*np.pi, n+1)
A = 4
a = 1
b = 2
xw = A*np.cos(a*t)
yw = A*np.sin(b*t)
Xpoint = []
Ypoint = []

def poseCallBack(data):
    global x
    global y
    global yaw
    x1  = data.pose.pose.orientation.x
    y1  = data.pose.pose.orientation.y
    z1 = data.pose.pose.orientation.z
    w1 = data.pose.pose.orientation.w
    yaw =quat2euler(x1,y1,z1,w1)[2]
    x=data.pose.pose.position.x
    y=data.pose.pose.position.y
    # yaw=pose.theta


def goal():
    # global x
    # global y
    # global yaw
    global Xpoint
    global Ypoint
    rospy.init_node('Kobuki_pose')
    cmd_vel_topic='/mobile_base/commands/velocity'
    velocity_pub = rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)
    position_topic ='/odom'
    pose_sub = rospy.Subscriber(position_topic,Odometry,poseCallBack)


    i=0
    rate=rospy.Rate(2)
    while not rospy.is_shutdown():
        velocity_msg=Twist()
        x_goal=xw[i]
        y_goal=yw[i]

        K_linear = 0.5
        distance = math.sqrt(((x_goal-x)**2)+((y_goal-y)**2))
        print(distance)
        velocity_msg.linear.x =distance*K_linear
        

        K_angular = 1
        angle_way=math.atan2(y_goal - y,x_goal - x)
        print("The angle is ",(angle_way-yaw),angle_way,yaw)
        d_theta = angle_way-yaw
        if (d_theta < -math.pi) :
            velocity_msg.angular.z=(angle_way-yaw + 2*math.pi)*K_angular
        else:
            velocity_msg.angular.z=(angle_way-yaw)*K_angular

        velocity_pub.publish(velocity_msg)
        if(distance<0.3):
            i=i+1
            Xpoint.append(x)
            Ypoint.append(y)
            #print(i)
            # print('x:',xw[i],' y:',yw[i])
        if(i==n):
            i=0
            #break #For only plotting purpose
        velocity_pub.publish(velocity_msg)
        rate.sleep()
  
    plt.plot(Xpoint,Ypoint,"b")
    plt.xlabel("XPoints")
    plt.ylabel("YPoints")
    plt.title("Odom Points")
    plt.show()

if __name__=='__main__':
    
    # '/mobile_base/commands/velocity'
    
    
    
    goal()


     
