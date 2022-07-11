import glob
import os
import sys
import time
import random

from lib import firebase_utils

# input configurations
config = {'Scenario': 'S1', 'Used_Case': 'UC1', 'duration': 30}

# retrieve data from firebase
dict_fr_list_retrieved = firebase_utils.retreive_data_from_firebase(config)
