import os
import sys
import time
import cv2
from datetime import datetime
import numpy as np

from lib.config_parameters import *

################################################## GENERAL FUNCTIONS ###############################################
def load_json(filename):
    if os.path.exists(filename):
        import json
        f = open(filename)
        data = json.load(f)
        f.close()
        if len(data)%500==1:
            print("- Previous data has "+str(len(data)) + " sensors recodrs")
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
    global json_path

    json_path = os.path.join(data_root, "missions")
    lidar_data_path = os.path.join(data_root, "sweeps", "LIDAR")
    road_image_path = os.path.join(data_root, "sweeps", "CSI_CAMERA")
    laneL_image_path = os.path.join(data_root, "sweeps", "LEFT_CAMERA")
    laneR_image_path = os.path.join(data_root, "sweeps", "RIGHT_CAMERA")
    surface_image_data_path = os.path.join(data_root,"sweeps","3D_CAMERA")

    create_new_folder(json_path)
    create_new_folder(lidar_data_path)
    create_new_folder(road_image_path)
    create_new_folder(laneL_image_path)
    create_new_folder(laneR_image_path)
    create_new_folder(surface_image_data_path)

def get_time_tag(type=1):
    from datetime import datetime
    today = datetime.now()
    
    if type == 0:
        return today.strftime("__%Y-%m-%d")
    else:
        return today.strftime("%Y-%m-%d-%Hh-%Mmin-%Ssec")

def get_sensor_filename(sensor_name, frame, tag=''):
    time_tag = str(get_time_tag(type=1))
    return time_tag  + "__" + sensor_name + "_"+ tag +"_" + str(frame)

def get_timestamp():
    # from datetime import datetime
    # # Getting the current date and time
    # dt = datetime.now()
    # # getting the timestamp
    # ts = datetime.timestamp(dt)
    import time
    ts=time.time()
    return ts

################################################## SAVINF SESORS FILES  ###############################################
def save_lidar_file(sensor_name, lidar_d ,sensor_frame):
    global data_root, dict_fr_list,  dict_frame, configuration, scene_count, car_location
    # save the colected data
    filename_strg=str(get_sensor_filename(sensor_name, sensor_frame))
    # # json
    # ext="json"
    # save_json_path=os.path.join(data_root, "sweeps", sensor_name, filename_strg + '.'+ext)
    # save_json([{"points": lidar_d}], save_json_path)
    
    #npy
    ext='npy'
    save_json_path=os.path.join(data_root, "sweeps", sensor_name, filename_strg + '.'+ext)
    np.save(save_json_path, lidar_d)

    #if disp:
    #    print(f'\n - Lidar data={lidar_d}')
    # update outputs
    dict_frame["description"]=configuration["description"]
    dict_frame["timestamp"]=get_timestamp()
    dict_frame["scene"]=scene_count
    dict_frame["sensor_name"]=sensor_name
    dict_frame["position"]={"Translation": car_location, 
                            "Rotation": []}
    dict_frame["calibration"]={"Translation": [], "Rotation": [], "Camera_intrinsic": ""}
    dict_frame["fileformat"]=ext
    dict_frame["filename"]=str(os.path.join("sweeps", sensor_name, filename_strg))
    dict_frame["meta_data"]=""
    
    return dict_frame

def save_image(sensor_name, frame, sensor_frame, tag=''):
    global  data_root, dict_fr_list, dict_frame, configuration, scene_count, car_location
    filename_strg=str(get_sensor_filename(sensor_name, sensor_frame, tag=tag)) + ".jpg"
    image_save_path=os.path.join(data_root, "sweeps", sensor_name, filename_strg)
    # save the fame in an image file
    cv2.imwrite(image_save_path, frame)
    
    dict_frame["description"]=configuration["description"]
    dict_frame["timestamp"]=get_timestamp()
    dict_frame["scene"]=scene_count
    dict_frame["sensor_name"]=sensor_name
    dict_frame["position"]={"Translation": car_location, 
                            "Rotation": []}
    dict_frame["calibration"]={"Translation": [], "Rotation": [], "Camera_intrinsic": ""}
    dict_frame["fileformat"]="jpg"
    dict_frame["filename"]=str(os.path.join("sweeps", sensor_name, filename_strg))
    dict_frame["meta_data"]=""

    return dict_frame


def save_3D_image(sensor_name, rgb_frame, depth_frame, sensor_frame):
    global  data_root, dict_fr_list, dict_frame, configuration, scene_count, car_location
    RGB_filename_strg=str(get_sensor_filename(sensor_name, sensor_frame)) + ".jpg"
    depth_filename_strg=RGB_filename_strg[:-4] + "_depth.png"
    # save the RGB  frame in an image file
    image_save_path=os.path.join(data_root, "sweeps", sensor_name, RGB_filename_strg)
    cv2.imwrite(image_save_path, rgb_frame)
    # save the RGB  frame in an image file
    image_save_path=os.path.join(data_root, "sweeps", sensor_name, depth_filename_strg)
    cv2.imwrite(image_save_path, depth_frame)
    
    dict_frame["description"]=configuration["description"]
    dict_frame["timestamp"]=get_timestamp()
    dict_frame["scene"]=scene_count
    dict_frame["sensor_name"]=sensor_name
    dict_frame["position"]={"Translation": car_location, 
                            "Rotation": []}
    dict_frame["calibration"]={"Translation": [], "Rotation": [], "Camera_intrinsic": ""}
    dict_frame["fileformat"]="jpg"
    dict_frame["filename"]=str(os.path.join("sweeps", sensor_name, RGB_filename_strg))
    dict_frame["meta_data"]={"depth_filename":depth_filename_strg}
    return dict_frame


def save_IMU_file(sensor_name, IMU_data):
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame, scene_count, car_location
    # dict_frame=add_data_to_pipeline(sensor_frame)
    dict_frame["description"]=configuration["description"]
    dict_frame["timestamp"]=get_timestamp()
    dict_frame["scene"]=scene_count
    dict_frame["sensor_name"]=sensor_name
    dict_frame["position"]={"Translation": car_location, 
                            "Rotation": []}
    dict_frame["calibration"]={"Translation": [], "Rotation": [], "Camera_intrinsic": ""}
    dict_frame["fileformat"]=""
    dict_frame["filename"]=""
    dict_frame["meta_data"]=IMU_data
    
    return dict_frame

def save_GPS_file(sensor_name, car_location, dict_GPS={}):
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame, scene_count
    # dict_frame=add_data_to_pipeline(sensor_frame)
    # sensor_frame=sensor_frame + 1
    
    dict_frame["description"]=configuration["description"]
    dict_frame["timestamp"]=get_timestamp()
    dict_frame["scene"]=scene_count
    dict_frame["sensor_name"]=sensor_name
    dict_frame["position"]={"Translation": car_location, 
                            "Rotation": []}
    dict_frame["calibration"]={"Translation": [], "Rotation": [], "Camera_intrinsic": ""}
    dict_frame["fileformat"]=""
    dict_frame["filename"]=""
    dict_frame["meta_data"]=dict_GPS
    
    return dict_frame

if __name__ == "__main__":
    for k in range(10):
        print(get_timestamp())
    


