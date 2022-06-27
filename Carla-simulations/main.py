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
from lib.carla_utils import *
from lib.firebase_utils import *


# input configurations
config = {'Scenario': 'S1', 'Used_Case': 'UC1', 'duration': 1000}
# print(config['Used_Case'])
#
#
# # Run CARLA Simulation
# simulation_parameters, dict_fr_list = utils_shashwat.run_carla_experiment(config)
#
#
# # Push data to firebase
# utils_shashwat.push_data_to_firebase(config, dict_fr_list, simulation_parameters)

# Retrieve data from firebase
dict_fr_list_retrieved = utils_shashwat.retreive_data_from_firebase(config)



