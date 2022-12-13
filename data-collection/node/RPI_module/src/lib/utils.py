import os
import sys
import time
import cv2
from datetime import datetime
import numpy as np

################################################## GENERAL FUNCTIONS ###############################################
def load_json(filename):
    if os.path.exists(filename):
        import json
        f = open(filename)
        data = json.load(f)
        f.close()
        if len(data)%500==1:
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
        filename_strg = time_tag + '_.json'
    else:
        time_tag = str(get_time_tag(type=1))
        filename_strg = 'Fr' + fr + '_' + time_tag + '_.json'
    dir_storage = ""
    return filename_strg, dir_storage

def create_new_folder(DIR):
  if not os.path.exists(DIR):
    os.makedirs(DIR)

def create_databse_folders(data_root):
    json_path = os.path.join(data_root, "missions")
    lidar_data_path = os.path.join(data_root, "sweeps", "LIDAR")
    road_image_path = os.path.join(data_root, "sweeps", "CSI_CAMERA")
    laneL_image_path = os.path.join(data_root, "sweeps", "LEFT_CAMERA")
    laneR_image_path = os.path.join(data_root, "sweeps", "RIGHT_CAMERA")
    create_new_folder(json_path)
    create_new_folder(lidar_data_path)
    create_new_folder(road_image_path)
    create_new_folder(laneL_image_path)
    create_new_folder(laneR_image_path)

def get_time_tag(type=1):
    from datetime import datetime
    today = datetime.now()
    if type == 0:
        return today.strftime("__%Y-%m-%d")
    else:
        return today.strftime("%Y-%m-%d-%Hh-%Mmin-%Ssec")

def get_sensor_filename(sensor_name, frame):
    time_tag = str(get_time_tag(type=1))
    return time_tag  + "__" + sensor_name + "__" + str(frame)

def get_timestamp():
    from datetime import datetime
    # Getting the current date and time
    dt = datetime.now()
    # getting the timestamp
    ts = datetime.timestamp(dt)
    return ts



if __name__ == "__main__":
    for k in range(10):
        print(get_timestamp())
    


