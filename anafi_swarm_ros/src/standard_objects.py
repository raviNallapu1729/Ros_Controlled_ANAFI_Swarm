# Olympe Imports
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing, Emergency, PCMD
from olympe.messages.ardrone3.SpeedSettings import MaxRotationSpeed
from olympe.messages.gimbal import set_max_speed, set_target

from olympe.messages.camera import (
    set_camera_mode,
    set_photo_mode,
    take_photo,
    photo_progress,
    recording_progress,
    set_recording_mode,
    set_exposure_settings,
    set_autorecord,
    stop_recording,
    start_recording

)

# Stabdard Imports
import time
import numpy as np
from termcolor import colored
import math
from math import sin, cos, atan2, sqrt, pi
import os
from os import mkdir
from datetime import datetime
import re
import requests
import shutil
import tempfile
import xml.etree.ElementTree as ET

# Custom Functions
from math_requirements import compute_yaw_Error, New_Sign, vec_mag, wrapTo2Pi


class Anafi_drone:
    def __init__(self, D_IP, nm):
        
        self.name = nm
        loc       = "/home/spacetrex/code/Logs/"
        tck       = str(round(time.time()))
        lf_nm     = loc+nm+tck
        self.D_IP = D_IP
        self.drone = olympe.Drone(self.D_IP, loglevel=4, logfile=open(lf_nm, "a+"))
        self.drone.connection()
        self.drone(stop_recording(cam_id=0)).wait()
        self.drone(set_camera_mode(cam_id=0, value="recording")).wait()

        self.loop_rate = 30.0                           # Set loop rate (Hz)
        self.X_tol     = 0.5                            # Position tolerance (m)
        self.V_tol     = 0.1                            # Velocity tolerance(m/s)
        self.yw_tol    = 2*pi/180                       # Yaw rate tolerance (rads)
        self.Tt_MX     = 10                             # Max tilt setting for yaw and pitch (deg)
        self.Tl_Mx     = 4                              # Max vertical speed (m/s)
        self.YR_MX     = 22                             # Max yaw rate (deg/s)
        self.GB_MX     = 50                             # Max gimbal rate (deg/s)
        self.thr_vec = [self.Tt_MX, self.Tt_MX, self.Tl_Mx, self.YR_MX] # Drone settings
        self.gn_mat = [3.6, 18, 4, 5, 9, 2]        # Controller Gains
        #self.gn_mat = [6, 18, 4, 5, 11, 2]
        self.ANAFI_IP = D_IP   # Real Drone

        # Drone web server URL
        self.ANAFI_URL = "http://{}/".format(self.ANAFI_IP)
        self.ANAFI_MEDIA_API_URL = self.ANAFI_URL + "api/v1/media/medias/"

        # Set Rotation Speed
        maxYawRate = self.drone(MaxRotationSpeed(self.YR_MX)).wait()
        if maxYawRate.success():
            print(colored(("Max Yaw rate set to: ", self.YR_MX),"green"))
        elif maxYawRate.timedout():
            print(colored(("Yaw rate action failed"),"green"))
        else:
            print(colored(("Yaw rate action in progress"),"green"))

        # Set Gimbal Speed
        max_GMB_Rate = self.drone(set_max_speed(gimbal_id=0, yaw = self.GB_MX/5, pitch = self.GB_MX, roll = self.GB_MX/5)).wait()
        if max_GMB_Rate.success():
            print(colored(("Max gimbal pitch rate set to: ", self.GB_MX),"green"))
        elif max_GMB_Rate.timedout():
            print(colored(("gimbal rate action failed"),"green"))
        else:
            print(colored(("gimbal rate action in progress"),"green"))

        
class Waypoint():
    def __init__(self, x, y, z, roll, pitch, yaw, vx, vy, vz, wx, wy, wz):
        self.x = x
        self.y = y
        self.z = z

        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

        self.vx = vx
        self.vy = vy
        self.vz = vz

        self.wx = wx
        self.wy = wy
        self.wz = wz


def drone_line(drone, X_St, X_Ref, gn_mat, thr_vec, X_tol):

    #  Drone State
    x_dr   = X_St[0]
    y_dr   = X_St[1]
    z_dr   = X_St[2]

    yaw_dr = X_St[8]

    # Reference
    x_R    = X_Ref[0]
    y_R    = X_Ref[1]
    z_R    = X_Ref[2]
    yaw_R  = X_Ref[8]

    Er = np.array(X_Ref) - np.array(X_St)
    r_er = vec_mag(Er[0:3])
    print(colored(("Position error: ", r_er),"red"))
    
    

    # UR, UP, UTh, UYr = PCMD_Controller(X_St, X_Ref, gn_mat, thr_vec)
    # drone(PCMD(1, UR, UP, UYr, UTh , 0))

    # drone.start_piloting()
    # drone.piloting_pcmd(UR, UP, UYr, UTh , 0)


    if r_er>=X_tol:

        UR, UP, UTh, UYr = PCMD_Controller(X_St, X_Ref, gn_mat, thr_vec)

        # drone(PCMD(1, UR, UP, UYr, UTh , 0))
        drone.start_piloting()
        drone.piloting_pcmd(UR, UP, UYr, UTh , 0)
        print(colored(("Drone position: ", x_dr, y_dr, z_dr, yaw_dr*180/pi),"yellow"))
        print(colored(("Target position: ", x_R, y_R, z_R, yaw_R*180/pi),"green"))
        print(colored(("Controller sent commands: ", UR, UP, UYr, UTh),"cyan"))

    # else:
        # print(colored(("Drone near target, no command sent"),"cyan"))
        # print(colored(("Drone position: ", x_dr, y_dr, z_dr, yaw_dr*180/pi),"yellow"))
        # print(colored(("Target position: ", x_R, y_R, z_R, yaw_R*180/pi),"green"))


def drone_center(drone, X_St, X_Ref, gn_mat, thr_vec, X_tol, yw_tol):

    #  Drone State
    x_dr   = X_St[0]
    y_dr   = X_St[1]
    z_dr   = X_St[2]

    vx_dr   = X_St[3]
    vy_dr   = X_St[4]
    vz_dr   = X_St[5]

    yaw_dr = X_St[8]

    # Reference
    x_R    = X_Ref[0]
    y_R    = X_Ref[1]
    z_R    = X_Ref[2]
    yaw_R  = X_Ref[8]

    Er   = np.array(X_Ref) - np.array(X_St)
    r_er = vec_mag(Er[0:3])
    Y_ER = compute_yaw_Error(yaw_R, yaw_dr)

    # print(colored(("Position error: ", r_er),"red"))
    # print(colored(("Yaw error: ", Y_ER),"red"))


    if r_er>=X_tol or abs(Y_ER)>=yw_tol:

        UR, UP, UTh, UYr = PCMD_Controller(X_St, X_Ref, gn_mat, thr_vec)
        
        # drone(PCMD(1, UR, UP, UYr, UTh , 0))
        drone.start_piloting()
        drone.piloting_pcmd(UR, UP, UYr, UTh , 0)

        
        # print(colored(("Drone position: ", x_dr, y_dr, z_dr, yaw_dr*180/pi),"yellow"))
        # print(colored(("Target position: ", x_R, y_R, z_R, yaw_R*180/pi),"green"))
        # print(colored(("Controller sent commands: ", UR, UP, UYr, UTh),"cyan"))

    else:
        print(colored(("Drone near target, no command sent"),"cyan"))
        print(colored(("Drone position: ", x_dr, y_dr, z_dr, yaw_dr*180/pi),"yellow"))
        print(colored(("Target position: ", x_R, y_R, z_R, yaw_R*180/pi),"green"))


def PCMD_Controller(state_Vec, ref_Vec, gn_mat, thr_vec):


    # Unchecked Control Commands

    Uxyz, Uyaw = R3_Mat_Drone_Commands(state_Vec, ref_Vec, gn_mat)
    Up_U  =    Uxyz[0]
    Ur_U  =    Uxyz[1]
    UTh_U =    Uxyz[2]
    Uyr_U =    Uyaw
    
    com_vec = np.array([Ur_U, Up_U, UTh_U, Uyr_U])

    # Obtain Saturated and capped commands
    Ur, Up, UTh, Uyr = check_PCMD_Sat(com_vec, thr_vec)

    return int(Ur), int(Up), int(UTh), int(Uyr)


def check_PCMD_Sat(com_vec, thr_vec):

    l = len(com_vec)
    ve = np.zeros(l)

    for i in range(l-1):
        if abs(com_vec[i]) <= thr_vec[i] and  abs(com_vec[i]) >= 0.01 :
            ve[i] = int(18*com_vec[i]/thr_vec[i])
        elif abs(com_vec[i]) < 0.01:
            ve[i] = 0
        else:
            ve[i] = int(18*New_Sign(com_vec[i]))

    i = l-1
    if abs(com_vec[i]) <= thr_vec[i]:
        ve[i] = int(85*com_vec[i]/thr_vec[i])
    else:
        ve[i] = int(85*New_Sign(com_vec[i]))


    Ur   = ve[0]
    Up   = ve[1]
    UTh  = ve[2]
    Uyr  = ve[3]

    return Ur, Up, UTh, Uyr


def R3_Mat_Drone_Commands(state_Vec, ref_Vec, gn_mat):
    #print(colored(ref_Vec, "red"))

    # Reference values
    x_R    = ref_Vec[0]
    y_R    = ref_Vec[1]
    z_R    = ref_Vec[2]
    vx_R   = ref_Vec[3]
    vy_R   = ref_Vec[4]
    vz_R   = ref_Vec[5]
    yaw_R  = ref_Vec[8]

    # Drone State
    x_dr   = state_Vec[0]
    y_dr   = state_Vec[1]
    z_dr   = state_Vec[2]

    vx_dr   = state_Vec[3]
    vy_dr   = state_Vec[4]
    vz_dr   = state_Vec[5]

    yaw_dr  = state_Vec[8]
    th_R1   = yaw_dr

    # Errors
    x_ER   = x_R - x_dr
    y_ER   = y_R - y_dr
    z_ER   = z_R - z_dr

    vx_ER   = vx_R - vx_dr
    vy_ER   = vy_R - vy_dr
    vz_ER   = vz_R - vz_dr
    yaw_ER = compute_yaw_Error(yaw_R, yaw_dr)

    # Gains
    g1 = gn_mat[0]
    g2 = gn_mat[1]
    g4 = gn_mat[3]
    g5 = gn_mat[4]
    g6 = gn_mat[5]

    # Holonomic  Controls
    Ux   =    (g1*x_ER + g2*vx_ER)
    Uy   =   -(g1*y_ER + g2*vy_ER)
    Uz   =     g4*z_ER + g5*vz_ER
    Uyaw =   -g6*yaw_ER*180/pi

    d_in = np.array([Ux, Uy, Uz])
    R3 = np.array([[cos(th_R1), -sin(th_R1), 0], [sin(th_R1), cos(th_R1), 0], [0, 0, 1]])
    dv2 = R3.dot(d_in)  # Drone frame commands

    return dv2, Uyaw


def gimbal_target(drone, abs_ang):
        drone(set_target(gimbal_id = 0, control_mode = "position", yaw_frame_of_reference = "absolute", yaw = 0, 
        pitch_frame_of_reference = "absolute", pitch = abs_ang,  roll_frame_of_reference ="absolute", roll = 0))


def R3_Mat_Drone_Commands_track(state_Vec, ref_Vec, gn_mat):
    #print(colored(ref_Vec, "red"))

    # Reference values
    x_R    = ref_Vec[0]
    y_R    = ref_Vec[1]
    z_R    = ref_Vec[2]
    vx_R   = ref_Vec[3]
    vy_R   = ref_Vec[4]
    vz_R   = ref_Vec[5]
    yaw_R  = ref_Vec[8]
    yaw_R  = ref_Vec[8]

    # Drone State
    x_dr   = state_Vec[0]
    y_dr   = state_Vec[1]
    z_dr   = state_Vec[2]

    vx_dr   = state_Vec[3]
    vy_dr   = state_Vec[4]
    vz_dr   = state_Vec[5]

    yaw_dr  = state_Vec[8]
    th_R1   = yaw_dr

    # Errors
    x_ER   = x_R - x_dr
    y_ER   = y_R - y_dr
    z_ER   = z_R - z_dr

    vx_ER   = vx_R - vx_dr
    vy_ER   = vy_R - vy_dr
    vz_ER   = vz_R - vz_dr
    yaw_ER = compute_yaw_Error(yaw_R, yaw_dr)

    # Gains
    g1 = gn_mat[0]
    g2 = gn_mat[1]
    g4 = gn_mat[3]
    g5 = gn_mat[4]
    g6 = gn_mat[5]

    # Holonomic  Controls
    Ux   =    (g1*x_ER + g2*vx_ER)
    Uy   =   -(g1*y_ER + g2*vy_ER)
    Uz   =     g4*z_ER + g5*vz_ER
    Uyaw =   -g6*yaw_ER*180/pi

    d_in = np.array([Ux, Uy, Uz])
    R3 = np.array([[cos(th_R1), -sin(th_R1), 0], [sin(th_R1), cos(th_R1), 0], [0, 0, 1]])
    dv2 = R3.dot(d_in)  # Drone frame commands

    return dv2, Uyaw


def gimbal_target(drone, abs_ang):
        drone(set_target(gimbal_id = 0, control_mode = "position", yaw_frame_of_reference = "absolute", yaw = 0, 
        pitch_frame_of_reference = "absolute", pitch = abs_ang,  roll_frame_of_reference ="absolute", roll = 0))



def filecreation():
    mydir = os.path.join(
        os.getcwd(), 
        datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
    try:
        os.makedirs(mydir)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise  # This was not a "directory exist" error
    return(mydir)

def drone_filter(X_St, X_Pr, eps_r, dt):

    R_P    = np.array(X_Pr[0:3])
    V_P    = np.array(X_Pr[3:6])
    Yaw_P  = np.array(X_Pr[8])
    w_P    = np.array(X_Pr[11])

    R_St   = np.array(X_St[0:3])

    Er     = R_P  - R_St
    r_er   = vec_mag(Er[0:3])

    if r_er >= eps_r:
        R_St  = R_P + V_P*dt
        Yaw   = wrapTo2Pi(Yaw_P + w_P*dt)
        X_St  = X_Pr
        X_St[0], X_St[1], X_St[2], X_St[8], = R_St[0], R_St[1], R_St[2], Yaw

    return X_St
    
     
