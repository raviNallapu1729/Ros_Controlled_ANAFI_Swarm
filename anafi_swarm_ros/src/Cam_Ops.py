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

import olympe
import os
from os import mkdir
from datetime import datetime
import re
import requests
import shutil
import tempfile
import xml.etree.ElementTree as ET
from math import pi, sin, cos
from numpy import arange
from olympe.messages import gimbal
from olympe.messages.ardrone3.Piloting import TakeOff, moveBy, Landing, PCMD
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, PositionChanged
from olympe.messages.ardrone3.SpeedSettings import MaxRotationSpeed
from olympe.messages.ardrone3.SpeedSettingsState import MaxRotationSpeedChanged
import numpy as np
from numpy import linspace

# Drone IP
ANAFI_IP = "192.168.42.1"   # Real Drone
#ANAFI_IP = "10.202.0.1"     # Sphinx Drone

# Drone web server URL
ANAFI_URL = "http://{}/".format(ANAFI_IP)
ANAFI_MEDIA_API_URL = ANAFI_URL + "api/v1/media/medias/"
XMP_TAGS_OF_INTEREST = (
        "CameraRollDegree",
        "CameraPitchDegree",
        "CameraYawDegree",
        "CaptureTsUs",
        # NOTE: GPS metadata is only present if the drone has a GPS fix  (i.e. they won't be present indoor)
        "GPSLatitude",
        "GPSLongitude",
        "GPSAltitude",
    )


def take_photo_burst_Move(drone, th_mov, th_R0, dxm):
    # take a photo burst and get the associated media_id

    xc, yc, zc =  R3_Mat_Cords(th_R0, dxm)
    photo_saved = drone(photo_progress(result="photo_saved", _policy="wait"))
    drone(take_photo(cam_id=0) & moveBy(xc, yc, zc, th_mov)).wait()
    photo_saved.wait()
    media_id = photo_saved.received_events().last().args["media_id"]

    # download the photos associated with this media id
    media_info_response = requests.get(ANAFI_MEDIA_API_URL + media_id)
    media_info_response.raise_for_status()
    os.chdir("/home/rnallapu/code/Results")
    download_dir = tempfile.mkdtemp()
    Res_dir = filecreation()

    #tempfile.gettempdir()
    for resource in media_info_response.json()["resources"]:
        image_response = requests.get(ANAFI_URL + resource["url"], stream=True)
        download_path = os.path.join(download_dir, resource["resource_id"])

        print('Hello')

        print(download_path)
        image_response.raise_for_status()

        with open(download_path, "wb") as image_file:
            shutil.copyfileobj(image_response.raw, image_file)

        # parse the xmp metadata
        with open(download_path, "rb") as image_file:
            image_data = image_file.read()
            image_xmp_start = image_data.find(b"<x:xmpmeta")
            image_xmp_end = image_data.find(b"</x:xmpmeta")
            image_xmp = ET.fromstring(image_data[image_xmp_start : image_xmp_end + 12])
            for image_meta in image_xmp[0][0]:
                xmp_tag = re.sub(r"{[^}]*}", "", image_meta.tag)
                xmp_value = image_meta.text
                # only print the XMP tags we are interested in
                if xmp_tag in XMP_TAGS_OF_INTEREST:
                    print(resource["resource_id"], xmp_tag, xmp_value)

    #
        #shutil.copy2(download_path, "/home/rnallapu/code/Photos/")
        shutil.copy2(download_path, Res_dir)



def take_Map_Video_Move(drone, th_mov, th_R0, dxm):
    # take a photo burst and get the associated media_id

    xc, yc, zc =  R3_Mat_Cords(th_R0, dxm)
    photo_saved = drone(recording_progress(result="stopped", _policy="wait"))
    # drone(
    #     MaxRotationSpeed(1)
    #     >> MaxRotationSpeedChanged(current=1, _policy='wait')
    # ).wait()
    #drone(setAutonomousFlightMaxRotationSpeed(1e-7)).wait()
    drone(start_recording(cam_id=0) & moveBy(xc, yc, zc, th_mov)).wait()
    drone(stop_recording(cam_id=0)).wait()

    print('Action Completed')
    photo_saved.wait()
    media_id = photo_saved.received_events().last().args["media_id"]
    print(media_id)

    # download the photos associated with this media id
    os.chdir("/home/rnallapu/code/Results")
    media_info_response = requests.get(ANAFI_MEDIA_API_URL + media_id)
    media_info_response.raise_for_status()
    

    download_dir = tempfile.mkdtemp()
    Res_dir = filecreation()
    

    #tempfile.gettempdir()
    for resource in media_info_response.json()["resources"]:
        image_response = requests.get(ANAFI_URL + resource["url"], stream=True)
        download_path = os.path.join(download_dir, resource["resource_id"])

        print('Hello')
        print(download_path)
        image_response.raise_for_status()

        with open(download_path, "wb") as image_file:
            shutil.copyfileobj(image_response.raw, image_file)

        #shutil.copy2(download_path, "/home/rnallapu/code/Photos/")
        shutil.copy2(download_path, Res_dir)


 #####################    Ravi Functions

def take_Map_Photo_Move(drone, th_mov, th_gim, th_R0,  xf, dxm, dzm):
    photo_saved = drone(photo_progress(result="photo_saved", _policy="wait"))

    # Pass - 1: 
    drone(moveBy(0, 0, dzm, 0) 
    >> FlyingStateChanged(state="hovering", _timeout=5)).wait() # Initial Height
    m_AR = []
    AMPU_ID = []
    
    drone(gimbal.set_target(
        gimbal_id=0,
        control_mode="position",
        yaw_frame_of_reference="relative",
        yaw=0.0,
        pitch_frame_of_reference="relative",
        pitch=th_gim,
        roll_frame_of_reference="relative",
        roll=0.0)
        >> olympe.messages.gimbal.attitude(pitch_relative=th_gim, _timeout=5)
    ).wait()
    
    D_th = th_mov 
    x_arr  = arange(0, xf, dxm)
    xlen   = len(x_arr)
    th_arr1 = linspace(0, D_th, xlen)
    th_arr  = th_arr1 + th_R0
    dth = th_arr1[1]

    # take a photo burst and get the associated media_id


    for ix in range(0, xlen-1):
        xc, yc, zc =  R3_Mat_Cords(th_arr[ix], dxm)
        drone(
            take_photo(cam_id=0) & moveBy(xc, yc, zc, dth)
        >> FlyingStateChanged(state="hovering", _timeout=5)).wait()
        photo_saved.wait()
        idp = photo_saved.received_events().last().args["media_id"]
        m_AR.append(idp)
        
    # Pass - 2: 
    drone(moveBy(0, 0, -2*dzm, 0)
    >> FlyingStateChanged(state="hovering", _timeout=5)).wait() # Initial Height
    
    drone(gimbal.set_target(
        gimbal_id=0,
        control_mode="position",
        yaw_frame_of_reference="relative",
        yaw=0.0,
        pitch_frame_of_reference="relative",
        pitch=-2*th_gim,
        roll_frame_of_reference="relative",
        roll=0.0)
        >> olympe.messages.gimbal.attitude(pitch_relative=-2*th_gim, _timeout=5)
    ).wait()
    
    D_th2 = -th_mov 
    th_arr2 = linspace(0, D_th2, xlen)
    dth2 = th_arr2[1]


    # take a photo burst and get the associated media_id
    for ix1 in range(0, xlen-1):
        xc, yc, zc =  R3_Mat_Cords(th_arr[xlen-ix1-1], -dxm)
        drone(
            take_photo(cam_id=0) & moveBy(xc, yc, zc, dth2)
        >> FlyingStateChanged(state="hovering", _timeout=5)).wait() 
        photo_saved.wait()
        idp = photo_saved.received_events().last().args["media_id"]
        m_AR.append(idp)   

    #media_id = photo_saved.received_events().last().args["media_id"]


    nph = len(m_AR)
    ## HEre

    # download the photos associated with this media id
    os.chdir("/home/rnallapu/code/Results")

    #media_info_response = requests.get(ANAFI_MEDIA_API_URL + m_AR)
    #media_info_response = requests.get(ANAFI_MEDIA_API_URL.join(m_AR))
    for inph in range(0, nph-1):
        AMPU_ID.append(ANAFI_MEDIA_API_URL + m_AR[inph])
    
    media_info_response = requests.get(AMPU_ID[2])
    media_info_response.raise_for_status()
    print(AMPU_ID)
    print('Action Completed')
    

    download_dir = tempfile.mkdtemp()
    Res_dir = filecreation()
    

    #tempfile.gettempdir()
    for resource in media_info_response.json()["resources"]:
        image_response = requests.get(ANAFI_URL + resource["url"], stream=True)
        download_path = os.path.join(download_dir, resource["resource_id"])

        print('Hello')
        print(download_path)
        image_response.raise_for_status()

        with open(download_path, "wb") as image_file:
            shutil.copyfileobj(image_response.raw, image_file)

        #shutil.copy2(download_path, "/home/rnallapu/code/Photos/")
        shutil.copy2(download_path, Res_dir)

######################## Functions End

def Setup_photo_burst_mode(drone):
    drone(stop_recording(cam_id=0)).wait()
    drone(set_autorecord(cam_id=0, state="inactive")).wait()
    drone(set_camera_mode(cam_id=0, value="photo")).wait()
    # For the file_format: jpeg is the only available option
    # dng is not supported in burst mode
    drone(
        set_exposure_settings(
            cam_id=0,
            mode="automatic_prefer_iso_sensitivity",
            shutter_speed="shutter_1_over_10000",
            iso_sensitivity="iso_1200",
            max_iso_sensitivity="iso_1200",
        )
    ).wait()


    drone(
        set_photo_mode(
            cam_id=0,
            mode="burst",
            format="rectilinear",
            file_format="jpeg",
            burst="burst_14_over_4s",
            bracketing="preset_1ev",
            capture_interval=0.0,
        )
    ).wait()

def Setup_Photo_mode(drone):
    drone(stop_recording(cam_id=0)).wait()
    drone(set_autorecord(cam_id=0, state="inactive")).wait()
    drone(set_camera_mode(cam_id=0, value="photo")).wait()
    # For the file_format: jpeg is the only available option
    # dng is not supported in burst mode
    drone(
        set_exposure_settings(
            cam_id=0,
            mode="automatic_prefer_iso_sensitivity",
            shutter_speed="shutter_1_over_10000",
            iso_sensitivity="iso_1200",
            max_iso_sensitivity="iso_1200",
        )
    ).wait()


    drone(
        set_photo_mode(
            cam_id=0,
            mode="single",
            format="rectilinear",
            file_format="jpeg",
            burst="burst_14_over_4s",
            bracketing="preset_1ev",
            capture_interval=0.0,
        )
    ).wait()



def Setup_Video_mode(drone):
    drone(stop_recording(cam_id=0)).wait()
    drone(set_camera_mode(cam_id=0, value="recording")).wait()
    drone(set_autorecord(cam_id=0, state="inactive")).wait()
    

    drone(
        set_recording_mode(
            cam_id=0,
            mode="standard",
            resolution="res_480p",
            framerate="fps_24",
            hyperlapse="ratio_15",
        )
    ).wait()



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


def R3_Mat_Cords(th_R0, dx):
    th_R1 = -th_R0
    R3 = np.array([[cos(th_R1), -sin(th_R1), 0], [sin(th_R1), cos(th_R1), 0], [0, 0, 1]])
    dv = np.array([dx, 0, 0])
    dv2 = R3.dot(dv)
    xc = dv2[0]
    yc = dv2[1]
    zc = dv2[2]
    return xc, yc, zc
    
