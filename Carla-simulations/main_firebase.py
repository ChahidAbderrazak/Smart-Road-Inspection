import glob
import os
import sys
import time
import random

from lib import firebase_utils


def main():
    # input configurations
    config = {'Scenario': 'S1', 'Used_Case': 'UC1', 'duration': 30}
    data_root='/media/abdo2020/DATA1/Datasets/data-demo/Carla-dataset'

    # retrieve data from firebase
    dict_fr_list_retrieved = firebase_utils.retreive_data_from_firebase(config, data_root=data_root)

if __name__ == '__main__':
    main()
