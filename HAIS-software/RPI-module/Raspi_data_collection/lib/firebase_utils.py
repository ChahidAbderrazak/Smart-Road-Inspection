import glob
import os
import sys
import time
import pickle
import pyrebase
import numpy as np
from lib import utils

dict_frame = {}  # dictionary of sensor data corresponding to a single frame
dict_fr_list = []  # list of individual dictionaries of sensors' data for all frames
data_root = "HAIS_DATABASE"  # storage path on local device
filenames_strg_list = []  # list of filenames stored on firebase corresponding to individual frames
configuration = {}  # input configuration
simulation_parameters = []
sensor_folders_names = ['Depth_Camera', 'Json_files', 'LIDAR', 'LIDAR_pcd', 'RADAR', 'RGB_Camera_Back',
                        'RGB_Camera_Front', 'SEMANTIC_LIDAR', 'SEMANTIC_LIDAR_pcd']
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


######################### GENERAL FUNCTIONS  #########################

def save_json(json_string, filename):
    """
    json_string=[{}, {}]
    filename="data/file.json"
    """
    import json
    try:
        # Using a JSON string
        with open(filename, "w") as outfile:
            json.dump(json_string, outfile, indent=2)
            return 0
    except:
        return 1


def load_json(filename):
    if os.path.exists(filename):
        import json
        f = open(filename)
        # print(filename)
        data = json.load(f)
        f.close()
    else:
        data = []
    return data


#########################  FIREBASE FUNCTIONS  ########################

def create_new_folder(DIR):
    if not os.path.exists(DIR):
        os.makedirs(DIR)


def create_database_folders():
    json_path = "D:/HAIS_DATABASE/sweeps/Json_files"
    lidar_data_path = "D:/HAIS_DATABASE/sweeps/LIDAR"
    radar_data_path = "D:/HAIS_DATABASE/sweeps/RADAR"
    semantic_lidar_data_path = "D:/HAIS_DATABASE/sweeps/SEMANTIC_LIDAR"
    rgb_camera_front_path = "D:\HAIS_DATABASE\sweeps\RGB_Camera_Front"
    rgb_camera_back_path = "D:\HAIS_DATABASE\sweeps\RGB_Camera_Back"
    depth_camera_path = "D:\HAIS_DATABASE\sweeps\Depth_Camera"
    semantic_segmentation_camera_path = "D:\HAIS_DATABASE\sweeps\Semantic_Segmentation_Camera"
    create_new_folder(json_path)
    create_new_folder(lidar_data_path)
    create_new_folder(radar_data_path)
    create_new_folder(semantic_lidar_data_path)
    create_new_folder(rgb_camera_front_path)
    create_new_folder(rgb_camera_back_path)
    create_new_folder(depth_camera_path)
    create_new_folder(semantic_segmentation_camera_path)


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


# download given filename from firebase
def download_file_from_firebase(filename_dir):
    try:
        filename = os.path.basename(filename_dir)
        folder_path = os.path.dirname(filename_dir)
        folder_list = []
        while True:
            folder = os.path.basename(folder_path)
            folder_list.append(folder)
            folder_path = os.path.dirname(folder_path)
            # print(folder_list)
            if folder_path == "":
                break

        # download filename
        json_filename = os.path.join(data_root, folder_list[-1], folder_list[-2], filename)
        storage.child("HAIS_DATA").child(str(folder_list[-1])).child(str(folder_list[-2])).child(str(filename)).download(json_filename)
        return 0

    except:
        print("error")
        return 1


def push_data_to_firebase():
    sensor_folders_list = []
    sweeps_dir = os.path.join(data_root, "sweeps")
    files = []

    for file in os.listdir(sweeps_dir):
        filename = os.fsdecode(file)
        sensor_folders_list.append(str(filename))

    for sensor_folder_name in sensor_folders_list:
        sensor_folder_dir = os.path.join(data_root, "sweeps", sensor_folder_name)
        for file in os.listdir(sensor_folder_dir):
            sensor_data_filename = os.fsdecode(file)
            files.append(str(sensor_data_filename))
            sensor_data_filename_dir = os.path.join(data_root, "sweeps", sensor_folder_name, str(sensor_data_filename))
            storage.child("HAIS_DATA").child("sweeps").child(sensor_folder_name).child(
                sensor_data_filename).put(sensor_data_filename_dir)


# Retrieve data from firebase

def retreive_data_from_firebase(attribute_name="sensor_name", value="SEMANTIC_LIDAR"):
    counter = 0

    # create folder to store data
    create_database_folders()

    # download json file from firebase
    json_filename = os.path.join(data_root, "sweeps", "Json_files", "SENSOR_DATA.json")
    storage.child("HAIS_DATA").child("sweeps").child("Json_files").child("SENSOR_DATA.json").download(json_filename)

    # load json file
    dict_fr_list = load_json(json_filename)

    for dict_ in dict_fr_list:
        if bool(dict_) == True:
            if dict_[attribute_name] == value:
                error = download_file_from_firebase(dict_["filename"])
                counter = counter + error
    print("Data downloaded with error: " + str(counter) + " failed download")

    # data_files_list_filename = os.path.join(data_root, "data_files_list.json")
    # create_database_folders()
    # data_files_list_retrieved = load_json(data_files_list_filename)
    # for dictionary in data_files_list_retrieved:
    #     for filename in dictionary["files"]:
    #         download_filename = os.path.join(data_root, "sweeps", dictionary["folder"], str(filename))
    #         storage.child("HAIS_DATA").child("sweeps").child(dictionary["folder"]).child(
    #             filename).download(download_filename)
