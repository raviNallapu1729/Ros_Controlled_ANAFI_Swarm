import numpy as np
import math
from math import pi

# 1. Convert Quaternion to Yaw, Pitch, Roll
def quat2angle(q):

    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2

    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = wrapTo2Pi( math.atan2(t3, t4) ) 

    return roll, pitch, yaw

# 2. Wrap angle between 0 to 2pi
def wrapTo2Pi(th):
    # This assumes th is an angle in [-pi, pi]

    th = th%(2*pi)
    
    return(th)

# 3. Compute Sign changes
def New_Sign(a):
    if a>=0.0:
        r = 1.0
    else:
        r = -1.0 
    return r


# 4. Compute angle difference between two angles in 0 to 2pi
def compute_yaw_Error(ywR, ywC):

    # 0. Initialize:
    er = [0, 0, 0, 0]
    
    # 1. Check for minimum rotation
    # Reference
    ywR1 = ywR
    ywR4 = ywR - (2*pi)
    # Current state
    ywC1 = ywC
    ywC4 = ywC - (2*pi)

    # 2. Compute Possible Errors   
    # E0, E1: Reference in Q1,2
    er[0] = ywR1 - ywC1
    er[1] = ywR1 - ywC4
    # E2, E3: Reference in Q3,4
    er[2] = ywR4 - ywC1
    er[3] = ywR4 - ywC4

    # 3. Find Absolute Values
    Er_mag = np.abs(er)
    
    # 4. Find Min Absolute Value
    imb = Er_mag.argmin() 

    # 5. Compute minimum Yaw rotation
    Yaw_Er = er[imb]

    # 6. Return
    return Yaw_Er

# Compute Magnitude
def vec_mag(v):
    mg = np.linalg.norm(v, ord=2)
    return mg


