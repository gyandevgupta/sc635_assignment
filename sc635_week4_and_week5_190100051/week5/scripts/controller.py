#!/usr/bin/env python
from __future__ import division
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
#from math import atan, atan2, pi
from quat2euler import quat2euler
import math
from week5.msg import Trilateration
from trilateration import varA, varB, varC, landmarkA, landmarkB, landmarkC

## Define global variables
pose = [0, 0, 0]
prev_pose = [0, 0, 0]
input1, input2 = 0.0, 0.0
Kp = 0.5
Kp_w = 0.2
## System noise related
noisy_pose = [0.0, 0.0, 0.0]
varTHETA, varX, varY = 0.01, 0.05, 0.05

Q = np.diag([varA, varB, varC]) ## Imported from trilateration
R = np.diag([varX, varY, varTHETA]) 
P = np.diag([0.5, 0.5, 0.5])  ## Some reasonable initial values

F = np.eye(3)                   ## System matrix for discretized unicycle is Identity 
H = np.zeros((3,2))             #get_current_H(pose, landmarkA, landmarkB, landmarkC)  ## H has to be calculated on the fly

distanceLandmarkA = 10.0
distanceLandmarkB = 10.0
distanceLandmarkC = 10.0

FILTER_ORDER= 5     ## Change filter settings
filter_a=[0 for i in range(FILTER_ORDER)]
filter_b=[0 for i in range(FILTER_ORDER)]
filter_c=[0 for i in range(FILTER_ORDER)]

idxA = 0
idxB = 0
idxC = 0

theta = 0

odoo = Odometry()



def get_current_H(pose, lA, lB, lC):
    ### Calculate the linearized measurement matrix H(k+1|k) at the current robot pose
    ## x and y co-ordinates of landmarks
    xA, yA = lA.x, lA.y
    xB, yB = lB.x, lB.y
    xC, yC = lC.x, lB.y
    x = pose[0]
    y = pose[1]
    H = np.zeros((3,3))
    H = [[x-xA,y-yA,0],[x-xB,y-yB,0],[x-xC,y-yC,0]] 
    H = np.array(H).reshape(3,3)
    print("The matrix H is, {}".format(H))

    return H

def dist(p1, p2):
    ### Given a pair of points the function returns euclidean distance
    return ( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 )**(0.5)

def predict_state(current_pose, current_v, current_w):
    ### System evolution 
    return noisy_pose

def predict_measurement(predicted_pose):
    ### Predicts the measurement (d1, d2, d3) given the current position of the robot
    global landmark_A, landmark_B, landmark_C
    d1 = dist(predicted_pose, landmark_A)
    d2 = dist(predicted_pose, landmark_B)
    d3 = dist(predicted_pose, landmark_C)
    measurement = [d1, d2, d3]
    measurement = np.array(measurement).reshape(3,1)
    return measurement


def callback2(data):
    global noisy_pose, varX, varY, varTHETA
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    noise = [np.random.normal(0,varX), np.random.normal(0,varY), np.random.normal(0,varTHETA)]
    noisy_pose = np.array([data.pose.pose.position.x + noise[0], data.pose.pose.position.y + noise[1], quat2euler(x,y,z,w)[2] + noise[2]]).reshape(3,1)



def callback(data):
    global distanceLandmarkA, distanceLandmarkB, distanceLandmarkC
    global idxA, idxB, idxC
    global filter_a, filter_b, filter_c
    global prev_pose, theta, pose
    global P, Q, R, F

    lA = data.landmarkA
    lB = data.landmarkB
    lC = data.landmarkC

    #######################################################
    ### FILTERING VALUES
    #######################################################
    ### Add value into r buffer at indices idxA, idxB, idxC
    filter_a[idxA] = lA.distance
    filter_b[idxB] = lB.distance
    filter_c[idxC] = lC.distance

    ### Increment indexes
    idxA += 1
    idxB += 1
    idxC += 1

    ### wrap around the indices if buffer full
    if idxA>=FILTER_ORDER:
        idxA=0
    if idxB>=FILTER_ORDER:
        idxB=0
    if idxC>=FILTER_ORDER:
        idxC=0

    ## Calculate filtered measurements (d1, d2, d3)  
    ### Locations of landmarks
    x1, y1 = lA.x, lA.y
    x2, y2 = lB.x, lB.y
    x3, y3 = lC.x, lC.y

    
    d1, d2, d3 = sum(filter_a)/FILTER_ORDER, sum(filter_b)/FILTER_ORDER, sum(filter_c)/FILTER_ORDER 

    Y_measured = np.matrix([[d1],[d2],[d3]])

    #### EXTENDED KALMAN FILTER CODE GOES BELOW

    ## Prediction: 
    #    you may use the function 'predict_state()' defined above as a substitute for prediction
    # x(k+1|k)
    State_prediction = predict_state(pose, input1, input2)
    
    ## Covariance update: 
    #    use the linearized state space matrix F to calculate the predicted covariance matrix
    # P(k+1|k)
    Cov_predict = np.dot(np.dot(F, P), np.transpose(F)) + Q
    ## Get measurement residual: 
    #    difference between y_measured and y_predicted of the (k+1)^th time instant
    diff = Y_measured - predict_measurement(State_prediction)
    ## Kalman gain calculation: 
    #    use the linearized measurement matrix 'H(k+1|k)' to compute the kalman gain
    # W(k+1) 
    H = get_current_H(pose, lA, lB, lC)

    S = np.dot(np.dot(H, Cov_predict), np.transpose(H)) + R
    ## Update state with kalman gain: 
    #    correct the predicted state using the measurement residual 
    # x(k+1|k+1)
    W = np.dot(np.dot(Cov_predict, np.transpose(H)), np.linalg.inv(S))
    pose = State_prediction + np.transpose(np.dot(W, diff))
    pose = np.transpose(pose)
    ## Update covariance matrix
    P = Cov_predict - np.dot(np.dot(W, S), np.transpose(W)) 
    # P(k+1|k+1)
    print("x:{}, \ty:{}, \ttheta:{}\n".format(pose[0], pose[1], theta*(180/pi)))

## Trajectory tracking business
def waypoint(t):
    x1 = math.cos(t*10*np.pi/180)
    y1 = math.sin(t*10*np.pi/180)
    return [x1,y1]

def angle(theta):
    if(theta > np.pi):
        theta = -(2*np.pi-theta)
    elif(theta < -np.pi):
        theta = (2*np.pi+theta)
    return theta
    
def control_loop():
    global input1, input2
    t = 0
    rospy.init_node('controller_node')
    pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    rospy.Subscriber('/trilateration_data', Trilateration, callback)
    rospy.Subscriber('/odom', Odometry, callback2)
    
    
    ## Setting the rate for loop execution
    rate = rospy.Rate(5) 
    
    ## Twist values to move the robot    
    err = 2.0
    rot = False 
    while not rospy.is_shutdown():
        velocity_msg = Twist()
        #### If robot has reached the current waypoint
        ####    Sample a new waypoint 
        #### Apply proportional control to reach the waypoint
        #### 
        [goalx, goaly]  = waypoint(t)
        err             = ((goalx-pose[0])**2 + (goaly-pose[1]**2))**0.5
        theta_ref       = math.atan2((goaly-pose[1]),(goalx-pose[0]))
        theta_err       = angle(theta_ref - pose[2])
        
        if(abs(theta_err) > (2.0*np.pi/180)):
            if rot:
                input2 = -Kp_w*theta_err
            else:
                input1  = 0.15
            rot = not rot
        else:
            input1  = Kp*err

        velocity_msg.linear.x = input1
        velocity_msg.angular.z = input2

        if err < 0.3:
            velocity_msg = Twist()
            t = t + 1
        pub.publish(velocity_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
    A = [[1,0,0],[0,1,0],[0,0,1]]
    A = np.array(A).reshape(3,3)
    print("The matrix A is, {}".format(A))