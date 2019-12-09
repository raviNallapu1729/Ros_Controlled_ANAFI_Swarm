#!/usr/bin/env python3

# Standard Imports
import rospy
import math
from math import sin, cos, atan2, acos, pi
import time
import numpy as np
from termcolor import colored
import sys
from signal import signal, SIGINT
import time
import os

# ROS imports
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# Custom Functions
from math_requirements import quat2angle, vec_mag, wrapTo2Pi    # Conversions
from Console_text import  Print_Drone_Actns          # Import Text Files
from standard_objects import *                       # Import Drone Class
from Drone_Ops import Drone_Actn                     # Import Temporary Dependencies


x = 0
y = 0
z = 0

vx = 0
vy = 0
vz = 0

roll = 0
pitch = 0
yaw = 0

qx = 0
qy = 0
qz = 0
qw = 1


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


def Drone_land(signal_recieved, frame):

    #drone.stop_piloting()
    time.sleep(3)
    Drone_Actn(11, drone)
    print("Simulation Exited")
    sys.exit(0)

def Drone_Line_Track(Dr_Obj, X_Ref):

    global x, y, z, vx, vy, vz, wx, wy, wz, roll, pitch, yaw

    # Pose Subscriber:
    Pose_Topic = "/vicon/anafi_1/odom"
    gn_mat  = Dr_Obj.gn_mat 
    thr_vec = Dr_Obj.thr_vec
    X_tol   = Dr_Obj.X_tol
    V_tol   = Dr_Obj.V_tol
    lr      = Dr_Obj.loop_rate
    looprate = rospy.Rate(lr) 

    while True:
        pose_subscriber = rospy.Subscriber(Pose_Topic, Odometry, poseCallback)
        X_St     = [x, y, z, vx, vy, vz, roll, pitch, yaw]
        Er = np.array(X_Ref) - np.array(X_St)
        r_er = vec_mag(Er[0:3])
        v_er = vec_mag(Er[3:6])

        drone_line(drone, X_St, X_Ref, gn_mat, thr_vec, X_tol)
        if r_er<=X_tol and v_er<=V_tol:
            print( colored( ("Drone Reached!"), 'green') )
            break
        looprate.sleep()

def Drone_Move_Orient(Dr_Obj, X_Ref):

    global x, y, z, vx, vy, vz, wx, wy, wz, roll, pitch, yaw

    # Pose Subscriber:
    Pose_Topic = "/vicon/anafi_1/odom"
    gn_mat  = Dr_Obj.gn_mat 
    thr_vec = Dr_Obj.thr_vec
    X_tol   = Dr_Obj.X_tol
    V_tol   = Dr_Obj.V_tol
    yw_tol  = Dr_Obj.yw_tol
    lr      = Dr_Obj.loop_rate
    looprate = rospy.Rate(lr) 

    
    while True:
        pose_subscriber = rospy.Subscriber(Pose_Topic, Odometry, poseCallback)
        X_St = [x, y, z, vx, vy, vz, roll, pitch, yaw]
        Er = np.array(X_Ref) - np.array(X_St)
        r_er = vec_mag(Er[0:3])
        v_er = vec_mag(Er[3:6])

        drone_center(drone, X_St, X_Ref, gn_mat, thr_vec, X_tol, yw_tol)
        if r_er<=X_tol and v_er<=V_tol:
            print( colored( ("Drone Reached!"), 'green') )
            break
        looprate.sleep()

def Drone_Center_Track(Dr_Obj, X_Ref):

    global x, y, z, vx, vy, vz, wx, wy, wz, roll, pitch, yaw

    # Pose Subscriber:
    Pose_Topic = "/vicon/anafi_1/odom"
    gn_mat  = Dr_Obj.gn_mat 
    thr_vec = Dr_Obj.thr_vec
    X_tol   = Dr_Obj.X_tol
    V_tol   = Dr_Obj.V_tol
    yw_tol  = Dr_Obj.yw_tol
    lr      = Dr_Obj.loop_rate
    looprate = rospy.Rate(lr) 

    
    while True:
        X_Ref[8] = wrapTo2Pi(atan2(-y, -x))
        pose_subscriber = rospy.Subscriber(Pose_Topic, Odometry, poseCallback)
        X_St = [x, y, z, vx, vy, vz, roll, pitch, yaw]
        Er = np.array(X_Ref) - np.array(X_St)
        r_er = vec_mag(Er[0:3])
        v_er = vec_mag(Er[3:6])

        drone_center(drone, X_St, X_Ref, gn_mat, thr_vec, X_tol, yw_tol)
        if r_er<=X_tol and v_er<=V_tol:
            print( colored( ("Drone Reached!"), 'green') )
            break
        looprate.sleep()

def Drone_Circle(Dr_Obj, X_Ref):

    global x, y, z, vx, vy, vz, wx, wy, wz, roll, pitch, yaw

    # Pose Subscriber:
    Pose_Topic = "/vicon/anafi_1/odom"
    gn_mat  = Dr_Obj.gn_mat 
    thr_vec = Dr_Obj.thr_vec
    X_tol   = Dr_Obj.X_tol
    V_tol   = Dr_Obj.V_tol
    yw_tol  = Dr_Obj.yw_tol
    lr      = Dr_Obj.loop_rate
    looprate = rospy.Rate(lr) 
    dt       = 1/lr 

    r_amp = 1.1
    Tp    = 30.0
    fr    = 2*pi/Tp

    print( colored( ("Going to Start !!"), 'cyan') )
    while True:
        pose_subscriber = rospy.Subscriber(Pose_Topic, Odometry, poseCallback)
        X_St = [x, y, z, vx, vy, vz, roll, pitch, yaw]
        Er = np.array(X_Ref) - np.array(X_St)
        r_er = vec_mag(Er[0:3])
        v_er = vec_mag(Er[3:6])

        drone_center(drone, X_St, X_Ref, gn_mat, thr_vec, X_tol, yw_tol)
        if r_er<=X_tol and v_er<=V_tol:
            print( colored( ("Drone Reached!"), 'green') )
            break
        looprate.sleep()


    print( colored( ("Starting Circle !!"), 'cyan') )
    for t in np.arange(0, 2*Tp, dt):
        pose_subscriber = rospy.Subscriber(Pose_Topic, Odometry, poseCallback)
        X_St = [x, y, z, vx, vy, vz, roll, pitch, yaw]
        X_Ref[0] = r_amp*cos(fr*t)
        X_Ref[1] = r_amp*sin(fr*t)
        X_Ref[3] = -r_amp*fr*sin(fr*t)
        X_Ref[4] =  r_amp*fr*cos(fr*t)

        X_Ref[8] = wrapTo2Pi(atan2(-y, -x))

        drone_center(drone, X_St, X_Ref, gn_mat, thr_vec, X_tol, yw_tol)

        looprate.sleep()

    print( colored( ("Circle commands sent !!"), 'green') )



if __name__ == '__main__':
    try:

        signal(SIGINT, Drone_land)

        rospy.init_node('anafi_interface', anonymous=True)

        # Pose Subscriber:
        Pose_Topic = "/vicon/anafi_1/odom"
        pose_subscriber = rospy.Subscriber(Pose_Topic, Odometry, poseCallback)

        # Initialize
        # Drone IP
        global drone
        Dr_IP = "192.168.42.1"  # Real Drone
        Dr_cl = Anafi_drone(Dr_IP)
        drone = Dr_cl.drone

        x0 = 11
        # Gains
        gn_mat = [3.5, 8.5, 4, 6, 1, 2]


        print('\x1bc')
        acn_N = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 20]
        acn   = ["Take off", "Mapping", "Go to 0,0,1.5", "Go Home and point out", "Track Center","Circle Origin","Move Up", "Move Down", "Yaw CW", "Yaw CCW", "Land", "Exit" ]
        n_actn = len(acn)

        while(x0<20):

            if x0<=n_actn:

                print('\x1bc')
                print( colored( ('Starting action: ' +  acn[x0-1] + '\n'), "green") )
                X_Ref = Drone_Actn(x0, drone)
                if x0==3:
                    Drone_Line_Track(Dr_cl, X_Ref)
                elif x0==4:
                    Drone_Move_Orient(Dr_cl, X_Ref)
                elif x0==5:
                    Drone_Center_Track(Dr_cl, X_Ref)
                elif x0==6:
                    Drone_Circle(Dr_cl, X_Ref)

                x0, c = Print_Drone_Actns(acn,  acn_N)

            else:
                print('\x1bc')
                print( colored( ('Invalid action selected, please select again! \n'), "red" ))
                x0, c = Print_Drone_Actns(acn,  acn_N)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")