import os
import sys
import time
import pygame
from datetime import datetime
from lib import brryIMU1_data
from picamera2 import Picamera2
from rplidar import RPLidar
import threading
import multiprocessing
import asyncio
import ast
import numpy as np
from lib import BrryGps1
stop_threads = False

dict_frame = {}  # dictionary of sensor data corresponding to a single frame
dict_fr_list = [{}]  # list of individual dictionaries of sensors" data for all frames
data_root = "/home/manir/hais/Shashwat/Hais_Data"  # storage path on local device
sensor_frame = 100000
configuration = {"Scenario": "S1", "USE_CASE": "UC1", "duration": 30,
                 "vehicle": "car1", "location": "Ontario Tech University",
                 "description": "low-safety index"}
PORT_NAME = '/dev/ttyUSB0'
picam2 = Picamera2()


################################################## GENERAL FUNCTIONS ###############################################

def load_json(filename):
    if os.path.exists(filename):
        import json
        f = open(filename)
        print(filename)
        data = json.load(f)
        f.close()
    else:
        data = []
    print(data)
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

# get json file name
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


def create_databse_folders():
    json_path = "/home/manir/hais/Shashwat/Hais_Data/sweeps/Json_files"
    lidar_data_path = "/home/manir/hais/Shashwat/Hais_Data/sweeps/LIDAR_pcd"
    image_data_path = "/home/manir/hais/Shashwat/Hais_Data/sweeps/CSI_CAMERA"

    create_new_folder(json_path)
    create_new_folder(lidar_data_path)
    create_new_folder(image_data_path)


def write_lidar_pcd(points, save_pcd_path):
    n = len(points)
    lines = []
    for i in range(n):
        x, y, z, intensity = points[i]
        lines.append('{} {} {} {}'.format(x, y, z, intensity))
    with open(save_pcd_path, 'w') as f:
        f.write(HEADER.format(n, n))
        f.write('\n'.join(lines))


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
    # Getting the current date and time
    dt = datetime.now()

    # getting the timestamp
    ts = datetime.timestamp(dt)
    return ts


# function to add data to main sensor data dictionary based on corresponding frame
def add_data_to_pipeline(frame):
    global dict_fr_list, dict_frame

    if dict_frame == {}:
        dict_frame["frame"] = frame

    elif frame != dict_frame["frame"]:
        dict_fr_list.append(dict_frame)  # main list with all frames
        #print(dict_fr_list)
        dict_frame = {}  # individual frame
        dict_frame["frame"] = frame

    return dict_frame


################################################## SENSOR FUNCTIONS #################################################

def save_lidar_data():
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame, stop_threads

    while True:
        '''Main function'''
        lidar = RPLidar(PORT_NAME)
        data = []
        try:
            print('Recording measurments... Press Crl+C to stop.')
            for scan in lidar.iter_scans():
                lidar_d = np.array(scan)
                filename_strg = str(get_sensor_filename("LIDAR", sensor_frame, configuration)) + ".pcd"
                save_pcd_path = os.path.join(data_root, "sweeps", "LIDAR_pcd", filename_strg)
                write_lidar_pcd(lidar_d, save_pcd_path)

                # update outputs
                dict_frame = add_data_to_pipeline(sensor_frame)
                sensor_frame = sensor_frame + 1
                dict_frame["description"] = configuration["description"]
                dict_frame["timestamp"] = get_timestamp()
                dict_frame["sensor_name"] = "LIDAR"
                dict_frame["position"] = {"Translation": "",
                                          "Rotation": ""}
                dict_frame["calibration"] = {"Translation": "", "Rotation": "", "Camera_intrinsic": ""}
                dict_frame["fileformat"] = "pcd"
                dict_frame["filename"] = str(os.path.join("sweeps", "LIDAR_pcd", filename_strg))
                dict_frame["meta_data"] = ""

        except KeyboardInterrupt:
            print('Stopping.')
        lidar.stop()
        lidar.stop_motor()
        lidar.reset()
        lidar.disconnect()

        if stop_threads:
            break
        
def save_json_file():
    global dict_fr_list, filename

    while True:
        if len(dict_fr_list)>5:       
            old_dict = load_json(filename)
            combined_dict = old_dict + dict_fr_list
            save_json(combined_dict, filename)
            print("No. of saved frames = " + str(len(combined_dict)))
            dict_fr_list = []

def save_image_data():
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame, stop_threads, picam2

    while True:
        # save image file locally
        filename_strg = str(get_sensor_filename("CSI_Camera", sensor_frame, configuration)) + ".jpg"
        image_save_path = os.path.join(data_root, "sweeps", "CSI_CAMERA", filename_strg)
        picam2.start_and_capture_file(image_save_path)

        # update outputs
        dict_frame = add_data_to_pipeline(sensor_frame)
        sensor_frame = sensor_frame + 1
        dict_frame["description"] = configuration["description"]
        dict_frame["timestamp"] = get_timestamp()
        dict_frame["sensor_name"] = "CSI_Camera"
        dict_frame["position"] = {"Translation": "",
                                  "Rotation": ""}
        dict_frame["calibration"] = {"Translation": "", "Rotation": "", "Camera_intrinsic": ""}
        dict_frame["fileformat"] = "jpg"
        dict_frame["filename"] = str(os.path.join("sweeps", "CSI_Camera", filename_strg))
        dict_frame["meta_data"] = ""


def save_accelerometer_data():
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame, stop_threads

    while True:    
        AccXangle, AccYangle, gyroXangle, gyroYangle, gyroZangle, CFangleX, CFangleY, heading, tiltCompensatedHeading, kalmanX, kalmanY = brryIMU1_data.get_GA_data()
        #print("Accelerometer" + str(pygame.time.get_ticks()))

        # update outputs
        dict_frame = add_data_to_pipeline(sensor_frame)
        sensor_frame = sensor_frame + 1
        dict_frame["description"] = configuration["description"]
        dict_frame["timestamp"] = get_timestamp()
        dict_frame["sensor_name"] = "IMU_SENSOR"
        dict_frame["position"] = {"Translation": "",
                                  "Rotation": ""}
        dict_frame["calibration"] = {"Translation": "", "Rotation": "", "Camera_intrinsic": ""}
        dict_frame["fileformat"] = ""
        dict_frame["filename"] = ""
        dict_frame["meta_data"] = {"ACCX Angle": AccXangle, "ACCY Angle": AccYangle, "GRYX Angle": gyroXangle,
                                   "GYRY Angle": gyroYangle, "GYRZ Angle": gyroZangle, "CFangleX Angle": CFangleX,
                                   "CFangleX Angle": CFangleY, "HEADING": heading,
                                   "tiltCompensatedHeading": tiltCompensatedHeading, "kalmanX": kalmanX, "kalmanY": kalmanY}
        #print(dict_frame)
        #print("accelerometer done")


def save_gps_data():
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame, stop_threads

    while pygame.time.get_ticks() < configuration["duration"] * 1000:
        print(pygame.time.get_ticks())
        print("1")
        gpsChars = str(BrryGps1.get_GPS_data())
        print("2")
        # update outputs
        dict_frame = add_data_to_pipeline(sensor_frame)
        sensor_frame = sensor_frame + 1
        dict_frame["description"] = configuration["description"]
        dict_frame["timestamp"] = get_timestamp()
        dict_frame["sensor_name"] = "GPS_SENSOR"
        dict_frame["position"] = {"Translation": "",
                                  "Rotation": ""}
        dict_frame["calibration"] = {"Translation": "", "Rotation": "", "Camera_intrinsic": ""}
        dict_frame["fileformat"] = ""
        dict_frame["filename"] = ""
        dict_frame["meta_data"] = gpsChars
    print("gps done")


def run_hais():
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame, stop_threads, picam2, filename

    filename_strg, dir_storage = utils.get_file_names(configuration)
    filename = os.path.join(data_root, "sweeps", "Json_files", filename_strg)
    save_json([{}], filename)
    # set up folder to save data locally
    utils.create_databse_folders()
    pygame.init()
    print("pygame initiated")
    t1 = threading.Thread(target=utils.save_json_file)
    t2 = threading.Thread(target=utils.save_image_data)
    t3 = threading.Thread(target=utils.save_accelerometer_data)

    t1.start()
    t2.start()
    t3.start()
    pygame.quit()

