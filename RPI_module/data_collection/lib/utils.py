import os
import sys
import time
import cv2
import pygame
from datetime import datetime
import numpy as np

# camera capture 
vid = cv2.VideoCapture(0)

################################################## GENERAL FUNCTIONS ###############################################
def load_json(filename):
    if os.path.exists(filename):
        import json
        f = open(filename)
        data = json.load(f)
        f.close()
        # if len(data)%500==1:
        print(f"- Previous data has {len(data)} sensors recodrs")
    else:
        data = []
    
    return data
    
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

# create json file
def get_file_names(config, fr=''):
    if fr == '':
        time_tag = str(get_time_tag(type=1))
        filename_strg = config['Scenario'] + '_' + config['USE_CASE'] + '_' + time_tag + '_.json'
    else:
        time_tag = str(get_time_tag(type=1))
        filename_strg = config['Scenario'] + '_' + config['USE_CASE'] + '_Fr' + fr + '_' + time_tag + '_.json'
    dir_storage = os.path.join(config['Scenario'], config['USE_CASE'])
    return filename_strg, dir_storage

def create_new_folder(DIR):
  if not os.path.exists(DIR):
    os.makedirs(DIR)

def create_databse_folders(data_root):
    json_path = os.path.join(data_root,"sweeps","Json_files")
    lidar_data_path = os.path.join(data_root,"sweeps","LIDAR")
    image_data_path = os.path.join(data_root,"sweeps","CSI_CAMERA")
    create_new_folder(json_path)
    create_new_folder(lidar_data_path)
    create_new_folder(image_data_path)

# def write_lidar_pcd(points, save_pcd_path):
#     n = len(points)
#     lines = []
#     for i in range(n):
#         x, y, z, intensity = points[i]
#         lines.append('{} {} {} {}'.format(x, y, z, intensity))
#     with open(save_pcd_path, 'w') as f:
#         f.write(HEADER.format(n, n))
#         f.write('\n'.join(lines))

def get_time_tag(type=1):
    from datetime import datetime
    today = datetime.now()
    if type == 0:
        return today.strftime("__%Y-%m-%d")
    else:
        return today.strftime("%Y-%m-%d-%Hh-%Mmin-%Ssec")

def get_sensor_filename(sensor_name, frame, config):
    try:
        scene_setup = "__" + config["Scenario"] + "-" + config["USE_CASE"]
    except:
        scene_setup = "__Unknown"
    time_tag = str(get_time_tag(type=1))
    return time_tag + scene_setup + "__" + sensor_name + "__" + str(frame)

def get_timestamp():
    from datetime import datetime
    # Getting the current date and time
    dt = datetime.now()

    # getting the timestamp
    ts = datetime.timestamp(dt)
    return ts





