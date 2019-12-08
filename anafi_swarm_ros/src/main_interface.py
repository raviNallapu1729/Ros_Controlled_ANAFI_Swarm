#!/usr/bin/env python3

import rospy
import math
from math import sin, cos, atan2, acos, pi
import time
import numpy as np
from termcolor import colored

# Standard imports
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# Conversions
from math_requirements import quat2angle


# Import Text Files
from Console_text import  Print_Drone_Actns

# Import Drone Class
from standard_objects import *

# Import Temporary Dependencies
from Drone_Ops import Drone_Actn
import time
import os


x = 0
y = 0
yaw = 0

def poseCallback(data):

    global x, y, z, qx, qy, qz, qw, vx, vy, vz, wx, wy, wz, roll, pitch, yaw

    # Position space
    # Translational
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z

    # Attitude
    qx = data.pose.pose.orientation.x
    qy = data.pose.pose.orientation.y
    qz = data.pose.pose.orientation.z
    qw = data.pose.pose.orientation.w
    q_vec = np.array([qx, qy, qz, qw])

    # Velocities
    # Linear
    vx = data.twist.twist.linear.x 
    vy = data.twist.twist.linear.y
    vz = data.twist.twist.linear.z

    # Angular
    wx= data.twist.twist.angular.x
    wy= data.twist.twist.angular.y
    wz= data.twist.twist.angular.z

    roll, pitch, yaw = quat2angle(q_vec)

    # rospy.loginfo("Pose recieved")
    # print( colored(("Yaw: ", yaw*180/pi), "yellow") ) 

    # spin() simply keeps python from exiting until this node is stopped



if __name__ == '__main__':
    try:
        rospy.init_node('anafi_interface', anonymous=True)

        # Pose Subscriber:
        Pose_Topic = "/vicon/anafi_1/odom"
        pose_subscriber = rospy.Subscriber(Pose_Topic, Odometry, poseCallback)


        # Initialize
        # Drone IP
        Dr_IP = "192.168.42.1"  # Real Drone
        Dr_cl = Anafi_drone(Dr_IP)
        drone = Dr_cl.drone

        x0 = 11


        print('\x1bc')
        acn_N = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 20]
        acn   = ["Take off", "Mapping", "Go to 0,0,1", "Move Back", "Move Left","Move Right","Move Up", "Move Down", "Yaw CW", "Yaw CCW", "Land", "Exit" ]
        n_actn = len(acn)

        while(x0<20):

            if x0<=n_actn:

                print('\x1bc')
                print( colored( ('Starting action: ' +  acn[x0-1] + '\n'), "green") )
                Drone_Actn(x0, drone)
                x0, c = Print_Drone_Actns(acn,  acn_N)

            else:
                print('\x1bc')
                print( colored( ('Invalid action selected, please select again! \n'), "red" ))
                x0, c = Print_Drone_Actns(acn,  acn_N)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")