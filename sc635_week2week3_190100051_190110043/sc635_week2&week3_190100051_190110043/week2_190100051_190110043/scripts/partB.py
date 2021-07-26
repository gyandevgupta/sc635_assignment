#!/usr/bin/env python
import math
import numpy
import rospy
from quat2euler import quat2euler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from matplotlib import pyplot as plt
import numpy as np

pose_x = 0.0
pose_y = 0.0
pose_t = 0.0
a = 1
b = 2
A = 4
totalPoints = 200
pos = []
ang = []

def callback(data):
    global pose_x
    global pose_y
    global pose_t
    x    = data.pose.pose.orientation.x;
    y    = data.pose.pose.orientation.y;
    z    = data.pose.pose.orientation.z;
    w    = data.pose.pose.orientation.w;
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
    for i in range(totalPoints+20):
        t.append(i*(2.0*math.pi)/(totalPoints))
    for i in range(totalPoints+20):
        X.append(A*math.cos(a*t[i]))
        Y.append(A*math.sin(b*t[i]))

def E_pos(point):
    dist = math.sqrt( ((point[0]-pose_x)**2) + ((point[1]-pose_y)**2) )
    return dist

def E_theta(point):
    dif = math.atan((point[1]-pose_y)/(point[0]-pose_x))
    return dif

def control():
    waypoint()
    global pos
    global ang
    pos = []
    ang = []
    rospy.init_node('kobuki')
    #pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, callback)
    i = 0
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        pos.append(E_pos((X[i],Y[i])))
        ang.append(E_theta((X[i],Y[i])))
        print("E_pos   = " + str(pos[i]) + ", E_theta = " + str(ang[i]))
        i = i + 1
        if(i == totalPoints):
            break
        rate.sleep()

    #Plotting the graph 
    xaxis = np.linspace(1,200,200)
    plt.plot(xaxis,pos,"r",label = "E_pos")
    plt.plot(xaxis,ang,"b",label = "E_theta")
    plt.xlabel("Data_Points")
    plt.ylabel("E_pos, E_theta")
    plt.title("E_pos, E_theta v/s Data_Points")
    plt.legend()
    plt.show()

if __name__ == '__main__':
    try:
        control()
    except rospy.ROSInterruptException:
        pass
