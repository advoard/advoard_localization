#!/usr/bin/env python3
'''
__author__ = "Bekir Bostanci"
__license__ = "BSD"
__version__ = "0.0.1"
__maintainer__ = "Bekir Bostanci"
__email__ = "bekirbostanci@gmail.com"
'''

import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist

import sys
import math
from threading import Timer
import time 

import numpy as np


global first_pose 
global second_pose 
first_pose = None
second_pose = None
global control_finish_move
control_finish_move = False
poses = []

poses_initial= []
poses_final= []
counter = 0 


def setInitialPosition(pose_ini_x,pose_ini_y,theta):
    #publish initialpose node 
    initial_pose_publisher = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1)
    initial_pose = PoseWithCovarianceStamped()
    
    initial_pose.header.seq = 1
    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.header.frame_id = "map"
    
    initial_pose.pose.pose.position.x = pose_ini_x
    initial_pose.pose.pose.position.y = pose_ini_y
    initial_pose.pose.pose.position.z = 0.0

    initial_pose.pose.pose.orientation.x = 0.0
    initial_pose.pose.pose.orientation.y = 0.0
    initial_pose.pose.pose.orientation.z = math.sin(theta/2)
    initial_pose.pose.pose.orientation.w = math.cos(theta/2)
    
    #standart covariance 
    initial_pose.pose.covariance = [0.24440499777841992, 0.0025508121327049984, 0.0, 0.0, 0.0, 0.0, 0.0025508121327049976, 0.24027264216307176, 0.0,0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.07042916431048865]
    
    #before the one message publishing we should sleep 1 second
    rospy.sleep(1)
    
    initial_pose_publisher.publish(initial_pose)

def subscribe_data_pose(PointStamped):
    global control_finish_move 
    #second uwb measurement set every update  
    if  poses_final == [] and len(poses_initial)<30:
            poses_initial.append([PointStamped.point.x,PointStamped.point.y])        
    elif control_finish_move == True and len(poses_final)<30 :
            poses_final.append([PointStamped.point.x,PointStamped.point.y])
            if len(poses_final) == 30:
                    finish_move()
                          

def move():
    global control_finish_move
    # Starts a new node
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()    
    #Receiveing the user's input
    speed =0.05
    distance =0.2
    vel_msg.linear.x = abs(speed)
    
    #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    
    current_distance = 0    
    
    while (current_distance < distance):
        rospy.sleep(0.01)
        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0    
        #Loop to move the turtle in an specified distance
        while(current_distance < distance):
            #Publish the velocity
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
        #After the loop, stops the robot
        vel_msg.linear.x = 0
        #Force the robot to stop
        velocity_publisher.publish(vel_msg)
    
    #After the loop, stops the robot
    vel_msg.linear.x = 0
    
    #Force the robot to stop
    velocity_publisher.publish(vel_msg)
    
    time.sleep(1)
    control_finish_move = True 

def finish_move():
    global first_pose 
    global second_pose 
    robot_radian=0
    
    
    a = np.array(poses_initial)
    b = np.array(poses_final)
    
    first_pose  = [a[:,0].mean() ,a[:,1].mean()]
    second_pose = [b[:,0].mean() ,b[:,1].mean()]
    #distance of first_pose and second_pose components'
    y_dif = b[:,1].mean() - a[:,1].mean()
    x_dif = b[:,0].mean() - a[:,0].mean()        
    robot_radian = math.atan2(y_dif,x_dif)
         
    result_log = [] 
    result_log.append("First Pose : "+ str(poses_initial))
    result_log.append("Second Pose : "+ str(poses_final))
    result_log.append("Degree Robot: "+ str(math.degrees(robot_radian)))
    rospy.loginfo(result_log)


    #publish initial pose with this information 
    setInitialPosition(second_pose[0],second_pose[1],robot_radian)

    #finish nodes
    rospy.signal_shutdown('initialpose_uwb_average')

if __name__ == "__main__":
    rospy.init_node('initialpose_uwb_average', anonymous=True)
    subscriber =rospy.Subscriber('localization_data_topic', PointStamped, subscribe_data_pose)
    
    t = Timer(3, move)
    t.start()
    
    rospy.spin()