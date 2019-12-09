# Olympe Imports
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing, Emergency, PCMD
from olympe.messages.ardrone3.SpeedSettings import MaxRotationSpeed

# Stabdard Imports
import numpy as np
from termcolor import colored
import math
from math import sin, cos, atan2, sqrt, pi

# Custom Functions
from math_requirements import compute_yaw_Error, New_Sign, vec_mag


class Anafi_drone:
    def __init__(self, D_IP):
        self.D_IP = D_IP
        self.drone = olympe.Drone(self.D_IP, loglevel=0)
        self.drone.connection()

        self.loop_rate = 10.0                           # Set loop rate (Hz)
        self.X_tol     = 0.5                            # Position tolerance (m)
        self.V_tol     = 0.1                            # Velocity tolerance(m/s)
        self.yw_tol    = 2*pi/180                       # Yaw rate tolerance (rads)
        self.Tt_MX     = 10                             # Max tilt setting for yaw and pitch (deg)
        self.Tl_Mx     = 4                              # Max vertical speed (m/s)
        self.YR_MX     = 22                             # Max yaw rate deg/s
        self.thr_vec = [self.Tt_MX, self.Tt_MX, self.Tl_Mx, self.YR_MX] # Drone settings
        self.gn_mat = [4.05, 8.6, 4, 6, 1, 2]        # Controller Gains

        maxYawRate = self.drone(MaxRotationSpeed(self.YR_MX)).wait()
        if maxYawRate.success():
            print(colored(("Yaw rate set to: ", self.YR_MX),"green"))
        elif maxYawRate.timedout():
            print(colored(("Yaw rate action failed"),"green"))
        else:
            print(colored(("Yaw rate action in progress"),"green"))

        
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

    vx_dr   = X_St[3]
    vy_dr   = X_St[4]
    vz_dr   = X_St[5]

    yaw_dr = X_St[8]


    # Reference
    x_R    = X_Ref[0]
    y_R    = X_Ref[1]
    z_R    = X_Ref[2]
    yaw_R  = X_Ref[8]

    Er = np.array(X_Ref) - np.array(X_St)
    r_er = vec_mag(Er[0:3])
    print(colored(("Position error: ", r_er),"red"))


    if r_er>=X_tol:

        UR, UP, UTh, UYr = PCMD_Controller(X_St, X_Ref, gn_mat, thr_vec)

        drone(PCMD(1, UR, UP, UYr, UTh , 0))
        print(colored(("Drone position: ", x_dr, y_dr, z_dr, yaw_dr*180/pi),"yellow"))
        print(colored(("Target position: ", x_R, y_R, z_R, yaw_R*180/pi),"green"))
        print(colored(("Controller sent commands: ", UR, UP, UYr, UTh),"cyan"))

    else:
        print(colored(("Drone near target, no command sent"),"cyan"))
        print(colored(("Drone position: ", x_dr, y_dr, z_dr, yaw_dr*180/pi),"yellow"))
        print(colored(("Target position: ", x_R, y_R, z_R, yaw_R*180/pi),"green"))


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

    Er = np.array(X_Ref) - np.array(X_St)
    r_er = vec_mag(Er[0:3])
    Y_ER = compute_yaw_Error(yaw_R, yaw_dr)

    print(colored(("Position error: ", r_er),"red"))
    print(colored(("Yaw error: ", Y_ER),"red"))


    if r_er>=X_tol or abs(Y_ER)>=yw_tol:

        UR, UP, UTh, UYr = PCMD_Controller(X_St, X_Ref, gn_mat, thr_vec)

        drone(PCMD(1, UR, UP, UYr, UTh , 0))
        print(colored(("Drone position: ", x_dr, y_dr, z_dr, yaw_dr*180/pi),"yellow"))
        print(colored(("Target position: ", x_R, y_R, z_R, yaw_R*180/pi),"green"))
        print(colored(("Controller sent commands: ", UR, UP, UYr, UTh),"cyan"))

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
            ve[i] = int(10*com_vec[i]/thr_vec[i])
        elif abs(com_vec[i]) < 0.01:
            ve[i] = 0
        else:
            ve[i] = int(10*New_Sign(com_vec[i]))

    i = l-1
    if abs(com_vec[i]) <= thr_vec[i]:
        ve[i] = int(80*com_vec[i]/thr_vec[i])
    else:
        ve[i] = int(80*New_Sign(com_vec[i]))


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