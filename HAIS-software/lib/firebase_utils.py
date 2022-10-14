import glob
import os
import sys
import time
import random
import pickle
import pyrebase
import numpy as np
from lib import utils


dict_frame = {}  # dictionary of sensor data corresponding to a single frame
dict_fr_list = []  # list of individual dictionaries of sensors' data for all frames
data_root = 'D:\HAIS_DATA'  # storage path on local device
filenames_strg_list = []  # list of filenames stored on firebase corresponding to individual frames
configuration = {}  # input configuration
simulation_parameters = []

flag = 1

# firebase storage configuration

firebaseConfig = {'apiKey': "AIzaSyCBTp3caunMJ6JlXyNXlN0DERVLi8EJ6Ho",
                  'authDomain': "hais-project-d9692.firebaseapp.com",
                  'databaseURL': "https://hais-project-d9692-default-rtdb.firebaseio.com",
                  'projectId': "hais-project-d9692",
                  'storageBucket': "hais-project-d9692.appspot.com",
                  'messagingSenderId': "926868745531",
                  'appId': "1:926868745531:web:6073d481701edc27c51b1e",
                  'measurementId': "G-25QKT2MGB6"}

firebase = pyrebase.initialize_app(firebaseConfig)
storage = firebase.storage()

#########################  FIREBASE FUNCTIONS  ########################
# convert pkl file to python dictionary
def pkl_to_dict(filename):
    open_file = open(filename, "rb")
    dict = pickle.load(open_file)
    open_file.close()
    return dict


# convert python dictionary to pkl file
def dict_to_pkl(filename, dict):
    """
     Save stored  variables list <var_list> in <filename>:
     save_variables(filename, var_list)
    """

    open_file = open(filename, "wb");
    pickle.dump(dict, open_file);
    open_file.close()


# get file name
def get_file_names(config, fr=''):
    if fr == '':
        filename_strg = config['Scenario'] + '_' + config['USE_CASE'] + '_.pkl'
    else:
        filename_strg = config['Scenario'] + '_' + config['USE_CASE'] + '_Fr' + fr + '_.pkl'
    dir_storage = os.path.join(config['Scenario'], config['USE_CASE'])
    return filename_strg, dir_storage


# Upload data on firebase
def push_data_to_firebase(config, dict_fr_list_push, simulation_parameters):
    global filenames_strg_list, flag, storage
    # print(f'\n config={config} \n dict_fr_list={dict_fr_list} ')

    # flag: if error occurs  in the next line, then break it into for loop 
    try:
        filename_strg, dir_storage = get_file_names(config)
        dict_to_pkl(filename_strg, dict_fr_list_push)
        dict_to_pkl('simulation_parameters.pkl', simulation_parameters)
        storage.child(config['Scenario']).child(config['USE_CASE']).child(filename_strg).put(filename_strg)
        storage.child(config['Scenario']).child(config['USE_CASE']).child('simulation_parameters.pkl').put(
            'simulation_parameters.pkl')
        flag = 1

    except:
        for dict in dict_fr_list_push:
            filename_strg, dir_storage = get_file_names(config, fr=str(dict['frame']))
            dict_to_pkl(filename_strg, dict)
            dict_to_pkl('simulation_parameters.pkl', simulation_parameters)
            filenames_strg_list.append(filename_strg)
            storage.child(config['Scenario']).child(config['USE_CASE']).child(filename_strg).put(filename_strg)
            storage.child(config['Scenario']).child(config['USE_CASE']).child('simulation_parameters.pkl').put(
                'simulation_parameters.pkl')
            flag = 0


# Retrieve data from firebase
def retreive_data_from_firebase(config, data_root='./data'):
    global filenames_strg_list, flag, storage
    dict_fr_list_retrieved = []

    # save reteived files in <filename>
    if flag == 1:
        filename_strg, dir_storage = get_file_names(config)
        filename = os.path.join(data_root, dir_storage, filename_strg)
        utils.create_new_folder(os.path.dirname(filename))
        storage.child(config['Scenario']).child(config['USE_CASE']).child(filename_strg).download(filename)
        dict_fr_list_retrieved = pkl_to_dict(filename)

    elif flag == 0:
        filename_strg, dir_storage = get_file_names(config)
        for file in filenames_strg_list:
            filename = os.path.join(data_root, dir_storage, file)
            utils.create_new_folder(os.path.dirname(filename))
            storage.child(config['Scenario']).child(config['USE_CASE']).child(filename_strg).download(filename)
            dict_frame_retrieved = pkl_to_dict(filename)
            dict_fr_list_retrieved.append(dict_frame_retrieved)

    return dict_fr_list_retrieved



