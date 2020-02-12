# Standard Imports
import os
import sys
import time
from math import pi, sin, cos
import re
import requests
import shutil
import tempfile
import xml.etree.ElementTree as ET
from termcolor import colored


# Olympe Imports
import olympe
from olympe.messages.ardrone3.GPSSettings import SetHome
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing, Emergency, PCMD
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, PositionChanged
from olympe.enums.ardrone3.PilotingState import FlyingStateChanged_State
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.ardrone3.PilotingSettings import setAutonomousFlightMaxRotationSpeed
from olympe.messages.ardrone3.SpeedSettings import MaxRotationSpeed
from olympe.messages.ardrone3.SpeedSettingsState import MaxRotationSpeedChanged



# ROS Imports
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

# Custom Imports
from math_requirements import vec_mag, quat2angle
from standard_objects import drone_line


def Drone_Actn(xa, drone):
	print(colored(('Your Message: ', xa), "blue"))
	X_Ref = []
	X_Home = [1, 2.5, 0.75, 0, 0, 0, 0, 0, 3*pi/2]

	if xa==1:
		
		if (drone.get_state(FlyingStateChanged)["state"] is not FlyingStateChanged_State.hovering):
			print(colored(('Initating Drone Take off !'), "green"))
			drone(TakeOff(_no_expect=True) & FlyingStateChanged(state="hovering", _policy="wait", _timeout=5)
			).wait()
			# drone.start_piloting()
		else:
			print(colored(('Drone has already taken off !'), "green"))


	if xa == 2:
		X_Ref = X_Home
		X_Ref[0] = 1
		X_Ref[2] = 0.75	

	elif xa==3:	
		X_Ref = [0, 0, 1.5, 0, 0, 0, 0, 0, 0]

	elif xa==4:
		X_Ref = X_Home

	elif xa==5:
		X_Ref = [1.2, -2.5, 1.3, 0, 0, 0, 0, 0, 0]

	elif xa==6:
		X_Ref = [1, 0, 1.5, 0, 0, 0, 0, 0, pi]

	elif xa==7:
		X_Ref = X_Home
	
	elif xa==8:
		X_Ref = X_Home

	elif xa==9:
		X_Ref = X_Home
		
	elif xa==10:
		drone(moveBy(0, 0, 0, -5*pi/180)
		>> FlyingStateChanged(state="hovering", _timeout=5)
		).wait()

	elif xa==11:
		print(colored(('Starting to Land !'), "green"))
		drone.stop_piloting()
		drone(Landing()).wait()
		time.sleep(5)
		drone(Emergency()).wait()
		print(colored(('Drone Landed !'), "green"))
		
	else:
		print('Currently Not programmed')
	
	return X_Ref




def Drone_Actns_2(xa, drone):
	
	X_Ref = []
	X_Home = [1.5,  2.7, 0.75, 0, 0, 0, 0, 0, 3*pi/2]
	X_End  = [1.5, -1.98, 0.75, 0, 0, 0, 0, 0, 3*pi/2]
	DX     = X_End[1] - X_Home[1]
	Tp     = 45
	Vy     = DX/Tp

	


	if xa==1:
		
		if (drone.get_state(FlyingStateChanged)["state"] is not FlyingStateChanged_State.hovering):
			print(colored(('Initating Drone Take off !'), "green"))
			drone(TakeOff(_no_expect=True) & FlyingStateChanged(state="hovering", _policy="wait", _timeout=5)
			).wait()
			# drone.start_piloting()
		else:
			print(colored(('Drone has already taken off !'), "green"))

	elif xa == 2:
		X_Ref = X_Home


	elif xa==3:	
		print(colored(('Holding Position!'), "green"))
		X_Ref = [0, 0, 1.5, 0, 0, 0, 0, 0, 0]

	elif xa==4:
		X_Ref = X_Home


	elif xa==5:
		print(colored(('Starting to Land !'), "green"))

		drone.stop_piloting()
		drone(Landing()).wait()
		time.sleep(5)

		drone(Emergency()).wait()

		print(colored(('Drone Landed !'), "green"))

	elif xa==6:
		print(colored(('Starting to Land !'), "green"))

		drone.stop_piloting()
		drone(Landing()).wait()
		time.sleep(5)

		drone(Emergency()).wait()

		print(colored(('Drone Landed !'), "green"))
		drone.disconnection()
		sys.exit(0)

	else:

		print(colored(('Your Message: ', xa), "blue"))
		print(colored(('Currently not programmed!'), "red"))
	
	return X_Ref, Tp, Vy, X_End

def Drone_Actns_3(xa, drone):
	
	X_Ref = []
	X_Home = [-1.5,  2.7, 0.75, 0, 0, 0, 0, 0, 3*pi/2]
	X_End  = [-1.5, -1.98, 0.75, 0, 0, 0, 0, 0, 3*pi/2]
	DX     = X_End[1] - X_Home[1]
	Tp     = 45
	Vy     = DX/Tp

	


	if xa==1:
		
		if (drone.get_state(FlyingStateChanged)["state"] is not FlyingStateChanged_State.hovering):
			print(colored(('Initating Drone Take off !'), "green"))
			drone(TakeOff(_no_expect=True) & FlyingStateChanged(state="hovering", _policy="wait", _timeout=5)
			).wait()
			# drone.start_piloting()
		else:
			print(colored(('Drone has already taken off !'), "green"))

	elif xa == 2:
		X_Ref = X_Home


	elif xa==3:	
		print(colored(('Holding Position!'), "green"))
		X_Ref = [0, 0, 1.5, 0, 0, 0, 0, 0, 0]

	elif xa==4:
		X_Ref = X_Home


	elif xa==5:
		print(colored(('Starting to Land !'), "green"))

		drone.stop_piloting()
		drone(Landing()).wait()
		time.sleep(5)

		drone(Emergency()).wait()

		print(colored(('Drone Landed !'), "green"))

	elif xa==6:
		print(colored(('Starting to Land !'), "green"))

		drone.stop_piloting()
		drone(Landing()).wait()
		time.sleep(5)

		drone(Emergency()).wait()

		print(colored(('Drone Landed !'), "green"))
		drone.disconnection()
		sys.exit(0)

	else:

		print(colored(('Your Message: ', xa), "blue"))
		print(colored(('Currently not programmed!'), "red"))
	
	return X_Ref, Tp, Vy, X_End


def Drone_settings(drone):
	drone(
		MaxRotationSpeed(20)
		>> MaxRotationSpeedChanged(current=1, _policy='wait')
	).wait()
