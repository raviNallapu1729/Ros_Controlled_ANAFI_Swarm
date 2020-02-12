#!/usr/bin/env python
# license removed for brevity

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
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int16
from threading import Thread
#import Console_text.py

status = 0

def anafi1(cmd):
    pub = rospy.Publisher("anafi_1/master", Int16, queue_size=10)
    data = Int16()
    data = cmd
    rospy.loginfo(data)
    pub.publish(data)

def anafi2(cmd):
    pub = rospy.Publisher("anafi_2/master", Int16, queue_size=10)
    data = Int16()
    data = cmd
    rospy.loginfo(data)
    pub.publish(data)

def anafi3(cmd):
    pub = rospy.Publisher("anafi_3/master", Int16, queue_size=10)
    data = Int16()
    data = cmd
    rospy.loginfo(data)
    pub.publish(data)


def anafi4(cmd):
    pub = rospy.Publisher("anafi_4/master", Int16, queue_size=10)
    data = Int16()
    data = cmd
    rospy.loginfo(data)
    pub.publish(data)

def anafi5(cmd):
    pub = rospy.Publisher("anafi_5/master", Int16, queue_size=10)
    data = Int16()
    data = cmd
    rospy.loginfo(data)
    pub.publish(data)


def timedMission():
        anafi1(3)
        time.sleep(1)
        anafi2(3)
        time.sleep(1)
        anafi3(3)
        time.sleep(1)

        anafi4(3)
        time.sleep(1)
        anafi5(3)
        time.sleep(1)


def Print_Drone_Actns(actn0, acn0_N):


	n_acn0 = len(actn0)
	cn = n_acn0-1

	print('#############################################################')
	print('  Hello, Welcome to the Anafi Swarm Control Interface!   ')
	print('############################################################# \n')
	print('Select a command from the following list: \n')
	for i in range(n_acn0):
		print(str(acn0_N[i]) + '-' + actn0[i])
	print('')
	x1 = int(input('Enter your command number:'))

	if x1<n_acn0:
		cn = acn0_N.index(x1)
	elif x1 == n_acn0:
		x1 = acn0_N[cn]
	return x1, cn

if __name__ == '__main__':


    '''Initialize all the drones'''
    rospy.init_node('MASTER', anonymous=True)


    while not rospy.is_shutdown():

        print('\x1bc')
        acn_N = [1, 2, 3, 4, 11, 20]
        acn   = ["Take Off", "Go Home", "Hover", "Mapping", "Land","Exit" ]
        n_actn = len(acn)

        x0, c = Print_Drone_Actns(acn,  acn_N)

        if x0<=n_actn:
            time.sleep(10)
            anafi1(x0)
            anafi2(x0)
            anafi3(x0)
            anafi4(x0)
            anafi5(x0)
        else:
            print('\x1bc')
            print( colored( ('Invalid action selected, please select again! \n'), "red" ))
        
        x0, c = Print_Drone_Actns(acn,  acn_N)
        print( colored( ('Starting action: ' +  acn[x0-1] + '\n'), "green") )
            
