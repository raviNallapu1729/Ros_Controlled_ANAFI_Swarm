#!/usr/bin/env python3

# Standard Imports
import rospy
import math
from math import sin, cos, atan2, acos, asin, pi
import time
import numpy as np
from termcolor import colored
from threading import Thread
import sys
from signal import signal, SIGINT
import time
import os
import re
import requests
import shutil
import tempfile
import xml.etree.ElementTree as ET

# ROS imports
import rospy
from std_msgs.msg import String, Int16
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# Custom Functions
from math_requirements import quat2angle, vec_mag, wrapTo2Pi    # Conversions
from Console_text import  Print_Drone_Actns          # Import Text Files
from standard_objects import *                       # Import Drone Class
from Drone_Ops import Drone_Actn                     # Import Temporary Dependencies

# Camera Functions
from olympe.messages.camera import (
    recording_progress,
    stop_recording,
    start_recording
)


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

    drone.stop_piloting()
    time.sleep(3)
    Drone_Actn(11, drone)
    print("Simulation Exited")
    sys.exit(0)

def Drone_Line_Track(Dr_Obj, X_Ref):

    global x, y, z, vx, vy, vz, wx, wy, wz, roll, pitch, yaw

    # Pose Subscriber:
    Pose_Topic = "/vicon/anafi_2/odom"
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
    Pose_Topic = "/vicon/anafi_2/odom"
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
    Pose_Topic = "/vicon/anafi_2/odom"
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
    Pose_Topic = "/vicon/anafi_2/odom"
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

    lr2      = 50
    looprate = rospy.Rate(lr2)
    dt       = 1/lr2

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


def Drone_Map_Opn(Dr_Obj, X_Ref, X_Tar, ran_V):

    thr_vec = Dr_Obj.thr_vec
    X_tol   = Dr_Obj.X_tol
    yw_tol  = Dr_Obj.yw_tol


    Drone_Move_Orient(Dr_Obj, X_Ref)
    print(colored( ("Moved home, executing map command!"), "green"))
    time.sleep(0.5)

    X_Ref2 = X_Ref
    y0     = X_Ref[1]

    global x, y, z, vx, vy, vz, wx, wy, wz, roll, pitch, yaw

    vyT = -0.15
    Tp  = 30

    lr       = 30
    looprate = rospy.Rate(lr)
    dt       = 1.0/lr

    gn_mat = [5, 15, 4, 4, 12, 1.8]        # Controller Gains
    X_tol   = 0.1


    for t in np.arange(0, Tp+dt, dt):

        pose_subscriber = rospy.Subscriber(Pose_Topic, Odometry, poseCallback)
        X_St = [x, y, z, vx, vy, vz, roll, pitch, yaw]

        X_Ref[1] = y0 + vyT*t
        X_Ref[4] =  vyT
        X_Ref[8] = wrapTo2Pi(atan2(-y, -x))

        DT = np.array(X_Tar) - np.array(X_St[0:3])
        r_tar = vec_mag(DT)
        th0 = asin(DT[2]/r_tar)*180/pi

        # print(colored( ("Commanded Pose: ", X_Ref[0], X_Ref[1], X_Ref[2],X_Ref[8]*(180/pi) ), "blue"))

        gimbal_target(drone, th0)
        drone_line(drone, X_St, X_Ref, gn_mat, thr_vec, X_tol)

        looprate.sleep()
    print(colored( ("Line command Finished !!" ), "blue"))
    drone.stop_piloting()
    time.sleep(0.5)
    X_Ref2[1] = -2.0
    Drone_Move_Orient(Dr_Obj, X_Ref2)
    print(colored( ("Mapping done !! Holding drone at Final Desitination!"), "green"))



def Drone_Map_Opn2(Dr_Obj, X_Ref, X_Tar, ran_V):

    thr_vec = Dr_Obj.thr_vec
    X_tol   = Dr_Obj.X_tol
    yw_tol  = Dr_Obj.yw_tol



    ANAFI_MEDIA_API_URL = Dr_Obj.ANAFI_MEDIA_API_URL
    ANAFI_URL           = Dr_Obj.ANAFI_URL


    Drone_Move_Orient(Dr_Obj, X_Ref)
    print(colored( ("Moved home, executing map command!"), "green"))
    time.sleep(0.5)

    X_Ref2 = X_Ref
    y0     = X_Ref[1]

    global x, y, z, vx, vy, vz, wx, wy, wz, roll, pitch, yaw
    # Pose_Topic = "/vicon/anafi_2/odom"

    vyT = -0.15
    Tp = 30

    lr       = 35
    looprate = rospy.Rate(lr)
    dt       = 1.0/lr
    X_tol   = 0.1

    gn_mat = [5, 15, 4, 4, 12, 1.8]        # Controller Gains
    rec_Vid = 0

    photo_saved = drone(recording_progress(result="stopped", _policy="wait"))

    for t in np.arange(0, Tp+dt, dt):

        pose_subscriber = rospy.Subscriber(Pose_Topic, Odometry, poseCallback)
        X_St = [x, y, z, vx, vy, vz, roll, pitch, yaw]

        X_Ref[1] = y0 + vyT*t
        X_Ref[4] =  vyT
        X_Ref[8] = wrapTo2Pi(atan2(-y, -x))

        DT = np.array(X_Tar) - np.array(X_St[0:3])
        r_tar = vec_mag(DT)
        th0 = asin(DT[2]/r_tar)*180/pi

        gimbal_target(drone, th0)
        drone_center(drone, X_St, X_Ref, gn_mat, thr_vec, X_tol, yw_tol)
        print(colored(("Distance to target: ", r_tar),"red"))
        print(colored(("Record Status: ", rec_Vid),"red"))
        print(colored(("Distance threshhold: ", 1.01*ran_V),"blue"))
        print(colored(("Gimbal angle computed: ", th0),"blue"))

        if r_tar<=1.01*ran_V and rec_Vid == 0:
            rec_Vid = 1
            drone(start_recording(cam_id=0))
            print(colored( ("Recording Started"), "blue"))

        elif r_tar>1.01*ran_V and rec_Vid == 1:
            rec_Vid = 2
            print(colored( ("Recording Completed"), "blue"))
            drone(stop_recording(cam_id=0))
            photo_saved.wait()

        looprate.sleep()
    time.sleep(0.5)


    # X_Ref2[1] = -2.0
    # Drone_Move_Orient(Dr_Obj, X_Ref2)
    drone(Landing()).wait()
    print(colored( ("Starting Transfer"), "green"))

    # photo_saved = drone(recording_progress(result="stopped", _policy="wait"))
    # photo_saved.wait()
    media_id = photo_saved.received_events().last().args["media_id"]
    print(colored( (media_id), "green"))

    os.chdir("/home/rnallapu/code/Results")
    media_info_response = requests.get(ANAFI_MEDIA_API_URL + media_id)
    media_info_response.raise_for_status()
    download_dir = tempfile.mkdtemp()
    Res_dir = filecreation()
        #tempfile.gettempdir()
    for resource in media_info_response.json()["resources"]:
        image_response = requests.get(ANAFI_URL + resource["url"], stream=True)
        download_path = os.path.join(download_dir, resource["resource_id"])
        print(colored( ("File Transfer in progress"), "blue"))

        image_response.raise_for_status()

        with open(download_path, "wb") as image_file:
            shutil.copyfileobj(image_response.raw, image_file)
        shutil.copy2(download_path, Res_dir)
        print(colored( ("File Transfer Completed !!!!!!!!!!"), "green"))
    print(colored( ("Mapping done !! Holding drone at Final Desitination!"), "green"))


def gimbal_track_moon(Dr_Obj, X_Ref, X_Tar):
    global x, y, z, vx, vy, vz, wx, wy, wz, roll, pitch, yaw
    # Pose Subscriber:
    Pose_Topic = "/vicon/anafi_2/odom"
    lr      = Dr_Obj.loop_rate
    X_tol   = Dr_Obj.X_tol
    V_tol   = Dr_Obj.V_tol
    looprate = rospy.Rate(lr)
    print( colored( ("Gimbal Action Started !!"), 'cyan') )

    while True:
        pose_subscriber = rospy.Subscriber(Pose_Topic, Odometry, poseCallback)
        X_St = [x, y, z, vx, vy, vz, roll, pitch, yaw]
        Er = np.array(X_Ref) - np.array(X_St)
        DT = np.array(X_Tar) - np.array(X_St[0:3])
        r_tar = vec_mag(DT)

        r_er = vec_mag(Er[0:3])
        v_er = vec_mag(Er[3:6])

        th0 = asin(DT[2]/r_tar)*180/pi
        # print(colored((th0), "red"))
        gimbal_target(drone, th0)


        if r_er<=X_tol and v_er<=V_tol:
            print( colored( ("Gimbal Action Completed!"), 'green') )
            break
        looprate.sleep()

def callback(data):
    global Dr_cl
    global drone

    #rospy.loginfo("DATA RECIEVED:", data)
    print("DATA:", data.data)
    x0 = data.data

    # Gains

    X_Tar  = [0, 0, 1.3]
    Vran = 1.33

    acn_N  = [1, 2, 3, 4, 11, 20]
    acn    = ["Take Off", "Go Home", "Hover", "Mapping", "Land","Exit" ]
    n_actn = len(acn)

    if x0<=n_actn:
        print( colored( ('Starting action: ' +  acn[x0-1] + '\n'), "green") )
        X_Ref = Drone_Actn(x0, drone)

        if x0==2:
            Drone_Map_Opn2(Dr_cl, X_Ref, X_Tar, Vran)
        elif x0==3:
            Drone_Line_Track(Dr_cl, X_Ref)
        elif x0==4:
            Drone_Move_Orient(Dr_cl, X_Ref)
        elif x0==5:
            Drone_Center_Track(Dr_cl, X_Ref)
        elif x0==6:
            Drone_Circle(Dr_cl, X_Ref)
        elif x0==7:
            gimbal_target(drone, 45)
        elif x0==8:
            gimbal_target(drone, -45)

    else:
        print( colored( ('Invalid action selected, please select again! \n'), "red" ))


def listener():
    rospy.init_node('anafi_2_listener', anonymous=True)

    #this topic needs to be changed for each computer
    rospy.Subscriber("anafi_2/master", Int16, callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        signal(SIGINT, Drone_land)
        global Dr_cl
        
        Dr_IP = "192.168.42.1"  # Real Drone
        Dr_cl = Anafi_drone(Dr_IP)
        drone = Dr_cl.drone
        x0    = 11
        X_Ref = Drone_Actn(x0, drone)

        listener()

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")
