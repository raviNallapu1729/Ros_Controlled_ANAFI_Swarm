from olympe.messages.camera import (
    set_camera_mode,
    set_photo_mode,
    take_photo,
    photo_progress,
)

from termcolor import colored
import olympe
import os
from os import mkdir
from datetime import datetime
import re
import requests
import shutil
import tempfile
import xml.etree.ElementTree as ET


# Drone IP
ANAFI_IP = "192.168.42.1"

# Drone web server URL
ANAFI_URL = "http://{}/".format(ANAFI_IP)

# Drone media web API URL
ANAFI_MEDIA_API_URL = ANAFI_URL + "api/v1/media/medias/"



def take_photo_burst(drone):
    # take a photo burst and get the associated media_id

    d_actn = drone(take_photo(cam_id=0)).wait()
    print(colored( (d_actn.success()), "cyan"))

    photo_saved = drone(photo_progress(result="photo_saved", _policy="wait"))
    photo_saved.wait()

    media_id = photo_saved.received_events().last().args["media_id"]
    print(colored( (media_id), "green"))

    os.chdir("/home/rnallapu/code/Results")

    # download the photos associated with this media id
    media_info_response = requests.get(ANAFI_MEDIA_API_URL + media_id)
    media_info_response.raise_for_status()


    download_dir = tempfile.mkdtemp()
    Res_dir = filecreation()


    for resource in media_info_response.json()["resources"]:
        image_response = requests.get(ANAFI_URL + resource["url"], stream=True)
        download_path = os.path.join(download_dir, resource["resource_id"])
        # download_path = os.path.join(Res_dir, resource["resource_id"])
        image_response.raise_for_status()

        print(colored( (download_dir), "red"))
        

        with open(download_path, "wb") as image_file:
            shutil.copyfileobj(image_response.raw, image_file)
    
        shutil.copy2(download_path, Res_dir)
        print(colored(("Photos Copied !!"), "cyan"))


def setup_photo_burst_mode(drone):
    drone(set_camera_mode(cam_id=0, value="photo")).wait()
    # For the file_format: jpeg is the only available option
    # dng is not supported in burst mode
    drone(
        set_photo_mode(
            cam_id=0,
            mode="burst",
            format="rectilinear",
            file_format="jpeg",
            burst="burst_14_over_1s",
            bracketing="preset_1ev",
            capture_interval=0.0,
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


def main(drone):
    drone.connection()
    setup_photo_burst_mode(drone)
    take_photo_burst(drone)


if __name__ == "__main__":
    with olympe.Drone(ANAFI_IP, loglevel=0) as drone:
        main(drone)