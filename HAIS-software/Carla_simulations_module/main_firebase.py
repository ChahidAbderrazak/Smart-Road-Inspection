import glob
import os
import sys
import time
import random
import numpy as np

from lib import firebase_utils
from lib import utils

def main():
    # input configurations
    config = {'Scenario': 'S1', 'USE_CASE': 'UC1', 'duration': 30, \
             'vehicle':'car1', 'location':'Ontario Tech University', \
             'description': 'low-safety index'}
    data_root='/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/Carla_scenes'
    # retrieve data from firebase
    dict_fr_listretrieved = firebase_utils.retreive_data_from_firebase(config, data_root=data_root)
    print(f'\n\n  -> Sample of the retreived data:\n\n{dict_fr_listretrieved[:12]} \
            \n\n  -> the data is stored in: \
            \n {data_root}')


if __name__ == '__main__':
    main()
