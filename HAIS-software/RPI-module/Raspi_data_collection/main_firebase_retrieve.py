import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

from lib import firebase_utils

# input configurations
config = {"Scenario": "S1", "USE_CASE": "UC1", "duration": 30,
          "vehicle": "car1", "location": "Ontario Tech University",
          "description": "low-safety index"}


# retrieve data from firebase
firebase_utils.retreive_data_from_firebase(attribute_name="sensor_name", value="Depth_Camera")





