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
import utils_shashwat
import syntax

# syntax.trial()
config = {'Scenario': 'S2', 'Used_Case': 'UC3', 'duration': 1000}
print(config['Used_Case'])
simulation_parameters, dict_fr_list = utils_shashwat.run_carla_experiment(config)
utils_shashwat.push_data_to_firebase(config, dict_fr_list, simulation_parameters)
# print(simulation_parameters)
# dict_fr_list_retrieved = utils_shashwat.retreive_data_from_firebase(config)
# print(dict_fr_list_retrieved)


