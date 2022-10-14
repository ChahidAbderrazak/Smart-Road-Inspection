import glob
import os
import sys
import time
import pygame
import random

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from lib import firebase_utils

# input configurations
config = {"Scenario": "S1", "USE_CASE": "UC1", "duration": 30,
          "vehicle": "car1", "location": "Ontario Tech University",
          "description": "low-safety index"}

# push data to firebase
firebase_utils.push_data_to_firebase()







