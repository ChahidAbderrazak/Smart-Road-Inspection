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
import syntax

# input configurations
config = {'Scenario': 'S1', 'Used_Case': 'UC1', 'duration': 30}

# retrieve data from firebase
dict_fr_list_retrieved = firebase_utils.retreive_data_from_firebase(config)
