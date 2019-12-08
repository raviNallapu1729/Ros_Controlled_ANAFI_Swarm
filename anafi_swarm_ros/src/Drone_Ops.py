from olympe.messages.camera import (
	set_camera_mode,
	set_photo_mode,
	take_photo,
	photo_progress,
)
import olympe
import os
import time
from math import pi, sin, cos
import re
import requests
import shutil
import tempfile
import xml.etree.ElementTree as ET

from olympe.messages.ardrone3.GPSSettings import SetHome
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing, Emergency, PCMD
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, PositionChanged
from olympe.enums.ardrone3.PilotingState import FlyingStateChanged_State
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages.ardrone3.PilotingSettings import setAutonomousFlightMaxRotationSpeed
from olympe.messages.ardrone3.SpeedSettings import MaxRotationSpeed
from olympe.messages.ardrone3.SpeedSettingsState import MaxRotationSpeedChanged
from Cam_Ops import *


def Drone_Actn(xa, drone):

	if xa==1:
		if (drone.get_state(FlyingStateChanged)["state"] is not FlyingStateChanged_State.hovering):
			drone(GPSFixStateChanged(fixed=1, _timeout=10, _policy="check_wait")).wait()
			drone(TakeOff(_no_expect=True) & FlyingStateChanged(state="hovering", _policy="wait", _timeout=5)
			).wait()
		else:
			print('Drone has already taken off !')

	if xa == 2:
		th_r0 = 30*pi/180
		th_mov = 55*pi/180
		th_gim = 15 # degrees
		xf     = 2.2
		dxm    = 0.3
		dzm    = 0.22

		drone(moveBy(0, 0, 0, th_r0)
		>> FlyingStateChanged(state="hovering", _timeout=5)
		).wait()

		#ca = take_photo_burst_Move(drone, th_mov, th_r0, xf)
		ca = take_Map_Video_Move(drone, th_mov, th_r0, xf)
		#ca = take_Map_Photo_Move(drone, th_mov, th_gim, th_r0,  xf, dxm, dzm)


	elif xa==3:
		# drone(moveBy(0.3, 0, 0, 0)
		# >> FlyingStateChanged(state="hovering", _timeout=5)
		# ).wait()

		drone(PCMD(1, 0, -10, 0, 0, 0)
		>> FlyingStateChanged(state="hovering", _timeout=5)
		).wait()
		print("PCMD Roll forward")

	elif xa==4:
		# drone(moveBy(-0.3, 0, 0, 0)
		# >> FlyingStateChanged(state="hovering", _timeout=5)
		# ).wait()

		drone(PCMD(1, 0, 10, 0, 0, 0)
		>> FlyingStateChanged(state="hovering", _timeout=5)
		).wait()
		print("PCMD Roll back")

	elif xa==5:
		# drone(moveBy(0, -1, 0, 0)
		# >> FlyingStateChanged(state="hovering", _timeout=5)
		# ).wait()

		drone(PCMD(1, -10, 0, 0, 0, 0)
		>> FlyingStateChanged(state="hovering", _timeout=5)
		).wait()

		print("PCMD Roll left")

	elif xa==6:
		# drone(moveBy(0, 1, 0, 0)
		# >> FlyingStateChanged(state="hovering", _timeout=5)
		# ).wait()

		drone(PCMD(1, 10, 0, 0, 0, 0)
		>> FlyingStateChanged(state="hovering", _timeout=5)
		).wait()

		print("PCMD Roll right")

	elif xa==7:
		# drone(moveBy(0, 0, -0.2, 0)
		# >> FlyingStateChanged(state="hovering", _timeout=5)
		# ).wait()

		drone(PCMD(1, 0, 0, 0, 20, 0)
		>> FlyingStateChanged(state="hovering", _timeout=5)
		).wait()

		print("PCMD way yaw 1")


	elif xa==8:
		# drone(moveBy(0, 0, 0.2, 0)
		# >> FlyingStateChanged(state="hovering", _timeout=5)
		# ).wait()

		drone(PCMD(1, 0, 0, 0, -20, 0)
		>> FlyingStateChanged(state="hovering", _timeout=5)
		).wait()

		print("PCMD way yaw 2")

	elif xa==9:
		drone(moveBy(0, 0, 0, 5*pi/180)
		>> FlyingStateChanged(state="hovering", _timeout=5)
		).wait()
		
	elif xa==10:
		drone(moveBy(0, 0, 0, -5*pi/180)
		>> FlyingStateChanged(state="hovering", _timeout=5)
		).wait()

	elif xa==11:
		drone(Landing()).wait()
		time.sleep(10)
		drone(Emergency()).wait()
	else:
		print('Currently Not programmed')


def Drone_settings(drone):
	drone(
		MaxRotationSpeed(1)
		>> MaxRotationSpeedChanged(current=1, _policy='wait')
	).wait()






