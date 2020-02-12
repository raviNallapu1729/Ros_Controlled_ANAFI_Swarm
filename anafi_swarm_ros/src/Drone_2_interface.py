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
import paramiko

# ROS imports
import rospy
from std_msgs.msg import String, Int16
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# Custom Functions
from math_requirements import quat2angle, vec_mag, wrapTo2Pi    # Conversions
from Console_text import  Print_Drone_Actns          # Import Text Files
from standard_objects import *                       # Import Drone Class
from Drone_Ops import Drone_Actns_2                  # Import Temporary Dependencies

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
    X_Ref = Drone_Actns_2(5, drone)
    
    print("Simulation Exited")
    sys.exit(0)


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


def Drone_Hover(Dr_Obj, X_Ref):

    global x, y, z, vx, vy, vz, wx, wy, wz, roll, pitch, yaw

    # Pose Subscriber:
    Pose_Topic = "/vicon/anafi_2/odom"
    gn_mat  = Dr_Obj.gn_mat
    thr_vec = Dr_Obj.thr_vec
    V_tol   = Dr_Obj.V_tol
    X_tol   = Dr_Obj.X_tol
    yw_tol  = Dr_Obj.yw_tol
    lr      = Dr_Obj.loop_rate
    looprate = rospy.Rate(lr)


    while True:
        pose_subscriber = rospy.Subscriber(Pose_Topic, Odometry, poseCallback)
        X_St = [x, y, z, vx, vy, vz, roll, pitch, yaw]
        Er = np.array(X_Ref) - np.array(X_St)
        v_er = vec_mag(Er[3:6])

        X_Ref[0:3] =  X_St[0:3]
        X_Ref[8]   =  X_St[8]

        drone_center(drone, X_St, X_Ref, gn_mat, thr_vec, X_tol, yw_tol)

        if v_er<=V_tol:
            print( colored( ("Drone Hovering!"), 'green') )
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


def Drone_Map_Opn2(Dr_Obj, X_Ref, X_Tar, ran_V, Tp, vyT, X_End):

    Mr_IP = '192.168.8.240'        # Master IP

    drone(stop_recording(cam_id=0))

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
    Pose_Topic = "/vicon/anafi_2/odom"

    lr       = 20

    print(colored(("Loop Rate: ", lr, " Hz"),"magenta"))
    print(colored(("Travel Time: ", Tp, " s"),"magenta"))
    print(colored(("Velocity: ", vyT, " m/s"),"magenta"))

    
    looprate = rospy.Rate(lr)
    dt       = 1.0/lr
    X_tol    = 0.1

    gn_mat = [5.5, 18, 4.3, 2, 12, 1.3]        # Controller Gains
    rec_Vid = 0

    photo_saved = drone(recording_progress(result="stopped", _policy="wait"))

    for t in np.arange(0, Tp+dt, dt):

        pose_subscriber = rospy.Subscriber(Pose_Topic, Odometry, poseCallback)
        X_St = [x, y, z, vx, vy, vz, roll, pitch, yaw]

        X_Ref[1] = y0 + vyT*t
        X_Ref[4] = vyT
        X_Ref[8] = wrapTo2Pi(atan2(-y, -x))

        DT = np.array(X_Tar) - np.array(X_St[0:3])
        r_tar = vec_mag(DT)
        th0 = asin(DT[2]/r_tar)*180/pi

        gimbal_target(drone, th0)
        drone_center(drone, X_St, X_Ref, gn_mat, thr_vec, X_tol, yw_tol)
        print(colored(("Distance to target: ", r_tar),"red"))
        print(colored(("Record Status: ", rec_Vid),"red"))
        print(colored(("Distance threshhold: ", 1.01*ran_V),"blue"))
        # print(colored(("Gimbal angle computed: ", th0),"blue"))

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

    print(colored( ("Current Time: ", t, "sec"),"cyan"))
    print(colored( ("Targetted End Y: ", X_Ref[1], "m/s"),"cyan"))
    print(colored( ("Achieved End Y: ", X_St[1], "m/s"),"cyan"))

    drone_center(drone, X_St, X_End, gn_mat, thr_vec, X_tol, yw_tol)
    drone(Landing()).wait()
    

    if rec_Vid==2:
        print(colored( ("Starting transfer to drone computer!"), "green"))
        media_id = photo_saved.received_events().last().args["media_id"]
        print(colored( (media_id), "green"))
        os.chdir("/home/spacetrex/code/Results")

        media_info_response = requests.get(ANAFI_MEDIA_API_URL + media_id)
        media_info_response.raise_for_status()
        download_dir         = tempfile.mkdtemp()
        Res_dir              = filecreation()
        #tempfile.gettempdir()
        for resource in media_info_response.json()["resources"]:
            image_response = requests.get(ANAFI_URL + resource["url"], stream=True)
            download_path = os.path.join(download_dir, resource["resource_id"])
            print(colored( ("File transfer to drone computer in progress"), "green"))

            image_response.raise_for_status()

            with open(download_path, "wb") as image_file:
                shutil.copyfileobj(image_response.raw, image_file)
            shutil.copy2(download_path, Res_dir)
            print(colored( ("File transfer to drone computer !!"), "green"))

        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(hostname= Mr_IP, username = 'spacetrex',password ='spacetrex',port= 22)
        sftp_client = ssh.open_sftp()
        loc_path = download_path
        rem_path = '/home/spacetrex/Results/ANAFI_2.MP4'

        print(colored( ("Starting transfer to Master Computer"), "cyan"))
        sftp_client.put(loc_path, rem_path)

        sftp_client.close()
        ssh.close()

        print(colored( ("File transfer to Master computer completed !!"), "cyan"))




    else:
        print(colored( ("No Recording obtained!!"), "red"))
    print(colored( ("Mapping done!! Holding drone at Final Desitination!"), "green"))




def Record_Transmit(Dr_Obj):

    Mr_IP               = '192.168.8.240'  # Master IP
    
    Setup_Video_mode(drone)
    drone(stop_recording(cam_id=0))
    
    ANAFI_MEDIA_API_URL = Dr_Obj.ANAFI_MEDIA_API_URL
    ANAFI_URL           = Dr_Obj.ANAFI_URL
    photo_saved         = drone(recording_progress(result="stopped", _policy="wait"))
    
    drone(start_recording(cam_id=0))
    print(colored( ("Recording Started"), "blue"))

    time.sleep(2)
    drone(stop_recording(cam_id=0))
    photo_saved.wait()

    print(colored( ("Recording Completed"), "blue"))

    print(colored( ("Starting transfer to Drone Computer"), "green"))
    media_id = photo_saved.received_events().last().args["media_id"]
    os.chdir("/home/spacetrex/code/Results")

    media_info_response = requests.get(ANAFI_MEDIA_API_URL + media_id)
    media_info_response.raise_for_status()
    download_dir        = tempfile.mkdtemp()
    Res_dir             = filecreation()

    for resource in media_info_response.json()["resources"]:
        image_response = requests.get(ANAFI_URL + resource["url"], stream=True)
        download_path = os.path.join(download_dir, resource["resource_id"])

        print(colored( ("File transfer to Drone Computer in progress"), "green"))

        image_response.raise_for_status()

        with open(download_path, "wb") as image_file:
            shutil.copyfileobj(image_response.raw, image_file)
        shutil.copy2(download_path, Res_dir)
        print(colored( ("File transfer to Drone computer completed !!"), "green"))


    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    ssh.connect(hostname= Mr_IP, username = 'spacetrex',password ='spacetrex',port= 22)
    sftp_client = ssh.open_sftp()

    #rem_path = '/home/spacetrex/Results/'+ media_id+'.MP4'
    rem_path = '/home/spacetrex/Results/ANAFI_2.MP4'
    loc_path = download_path


    print(colored( ("Starting transfer to Master Computer"), "cyan"))
    sftp_client.put(loc_path, rem_path)
   
    sftp_client.close()
    ssh.close()
    print(colored( ("File transfer to Master computer completed !!"), "cyan"))


def callback(data):
    global Dr_cl
    global drone

    #rospy.loginfo("DATA RECIEVED:", data)
    print('\x1bc')
    print("DATA:", data.data)
    x0 = data.data

    # Gains

    X_Tar  = [0, 0, 1.3]
    Vran = 1.7

    acn_N  = [1, 2, 3, 4, 5, 6, 7]
    acn    = ["Take Off", "Go Home", "Hover", "Mapping", "Land", "Exit", "Record Video" ]
    n_actn = len(acn)

    if x0<=n_actn:
        
        print( colored( ('Starting action: ' +  acn[x0-1] + '\n'), "green") )
        X_Ref, Tp, vyT, X_End = Drone_Actns_2(x0, drone)

        if x0==2:
            Drone_Move_Orient(Dr_cl, X_Ref)

        elif x0==3:
            Drone_Hover(Dr_cl, X_Ref)

        elif x0==4:
            Drone_Map_Opn2(Dr_cl, X_Ref, X_Tar, Vran, Tp, vyT, X_End)
        
        elif x0==7:
            Record_Transmit(Dr_cl)

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
        Setup_Video_mode(drone)
        drone(stop_recording(cam_id=0))
        x0    = 5
        print('\x1bc')
        X_Ref = Drone_Actns_2(x0, drone)

        listener()

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")
