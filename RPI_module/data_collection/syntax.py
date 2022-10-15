import os
import sys
import time
import signal
import pygame
import threading
from lib.config_parameters import *
from lib.utils import *
from lib import Gps, Imu
from lib.lidar_sensor import *
# from rplidar import RPLidar
       
################################################## SENSOR FUNCTIONS #################################################
def save_lidar_data(lidar_d):
    global img_size, data_root, dict_fr_list, dict_frame, configuration, sensor_frame, stop_threads, vid, lidar_device,  car_location

    filename_strg = str(get_sensor_filename("LIDAR", sensor_frame, configuration)) + ".json"
    save_json_path = os.path.join(data_root, "sweeps", "LIDAR", filename_strg)
    save_json([{"points": lidar_d}], save_json_path)

    #if disp:
    #    print(f'\n - Lidar data={lidar_d}')
    # update outputs
    dict_frame = add_data_to_pipeline(sensor_frame)
    sensor_frame = sensor_frame + 1
    dict_frame["description"] = configuration["description"]
    dict_frame["timestamp"] = get_timestamp()
    dict_frame["sensor_name"] = "LIDAR"
    dict_frame["position"] = {"Translation": car_location,
                            "Rotation": ""}
    dict_frame["calibration"] = {"Translation": "", "Rotation": "", "Camera_intrinsic": ""}
    dict_frame["fileformat"] = "json"
    dict_frame["filename"] = str(os.path.join("sweeps", "LIDAR", filename_strg))
    dict_frame["meta_data"] = ""
                
def save_json_file():
    global img_size, dict_fr_list, filename
    if len(dict_fr_list)>5:    
        old_dict = load_json(filename)
        combined_dict = old_dict + dict_fr_list
        save_json(combined_dict, filename)
        print(f"\n - Saving {str(len(combined_dict))}  frames in {filename}" )
        dict_fr_list = []

def save_image_data():
    global img_size, data_root, dict_fr_list, dict_frame, configuration, sensor_frame, stop_threads, vid,  car_location
    # save image file locally
    filename_strg = str(get_sensor_filename("CSI_CAMERA", sensor_frame, configuration)) + ".jpg"
    image_save_path = os.path.join(data_root, "sweeps", "CSI_CAMERA", filename_strg)
    ret, frame = vid.read()
    #frame = cv2.resize(frame, img_size) 
    cv2.imwrite(image_save_path, frame)
    # update outputs
    dict_frame = add_data_to_pipeline(sensor_frame)
    sensor_frame = sensor_frame + 1
    dict_frame["description"] = configuration["description"]
    dict_frame["timestamp"] = get_timestamp()
    dict_frame["sensor_name"] = "CSI_Camera"
    dict_frame["position"] = {"Translation": car_location,
                              "Rotation": ""}
    dict_frame["calibration"] = {"Translation": "", "Rotation": "", "Camera_intrinsic": ""}
    dict_frame["fileformat"] = "jpg"
    dict_frame["filename"] = str(os.path.join("sweeps", "CSI_Camera", filename_strg))
    dict_frame["meta_data"] = ""

def save_accelerometer_data():
    global img_size,  data_root, dict_fr_list, dict_frame, configuration, sensor_frame, stop_threads, car_location

    AccXangle, AccYangle, gyroXangle, gyroYangle, gyroZangle, CFangleX, CFangleY, heading, tiltCompensatedHeading, kalmanX, kalmanY = Imu.imuDt()
    #print("Accelerometer" + str(pygame.time.get_ticks()))

    # update outputs
    dict_frame = add_data_to_pipeline(sensor_frame)
    sensor_frame = sensor_frame + 1
    dict_frame["description"] = configuration["description"]
    dict_frame["timestamp"] = get_timestamp()
    dict_frame["sensor_name"] = "IMU_SENSOR"
    dict_frame["position"] = {"Translation": car_location,
                              "Rotation": ""}
    dict_frame["calibration"] = {"Translation": "", "Rotation": "", "Camera_intrinsic": ""}
    dict_frame["fileformat"] = ""
    dict_frame["filename"] = ""
    dict_frame["meta_data"] = {"ACCX Angle": AccXangle, "ACCY Angle": AccYangle, "GRYX Angle": gyroXangle,
                               "GYRY Angle": gyroYangle, "GYRZ Angle": gyroZangle, "CFangleX Angle": CFangleX,
                               "CFangleX Angle": CFangleY, "HEADING": heading,
                               "tiltCompensatedHeading": tiltCompensatedHeading, "kalmanX": kalmanX, "kalmanY": kalmanY}

def save_gps_data():
    global img_size,  data_root, dict_fr_list, dict_frame, configuration, sensor_frame, stop_threads, car_location
    lat, lng, alt = [-1,-1,-1]# Gps.gpsDt()
    car_location=[lat, lng, alt]
    print(f"\n - car_location ={car_location}")
    

    # update outputs
    dict_frame = add_data_to_pipeline(sensor_frame)
    sensor_frame = sensor_frame + 1
    dict_frame["description"] = configuration["description"]
    dict_frame["timestamp"] = get_timestamp()
    dict_frame["sensor_name"] = "GPS_SENSOR"
    dict_frame["position"] = {"Translation": car_location,
                              "Rotation": ""}
    dict_frame["calibration"] = {"Translation": "", "Rotation": "", "Camera_intrinsic": ""}
    dict_frame["fileformat"] = ""
    dict_frame["filename"] = ""
    dict_frame["meta_data"] = ""

def add_data_to_pipeline(frame):
    # function to add data to main sensor data dictionary based on corresponding frame
    global img_size,  dict_fr_list, dict_frame
    if dict_frame == {}:
        dict_frame["frame"] = frame

    elif frame != dict_frame["frame"]:
        dict_fr_list.append(dict_frame)  # main list with all frames
        #print(dict_fr_list)
        dict_frame = {}  # individual frame
        dict_frame["frame"] = frame
    return dict_frame

def init():
    global car_location, sensor_frame, disp
    disp=False
    dict_fr_list = []
    # get the car position
    lat, lng, alt = [-1,-1,-1]# Gps.gpsDt()
    car_location=[lat, lng, alt]

    # define the log file
    data_log = load_json(filename)
    sensor_frame = 1+len(data_log)
    if sensor_frame==0:
        save_json([{}], filename)
    # set up folder to save data locally
    create_databse_folders(data_root)
    # display
    print(f"configuration={configuration}")
    print(f"filename={filename}")
    print(f"sensor_frame={sensor_frame}")
    
    

def collect_node_data():
    global img_size, data_root, dict_fr_list, dict_frame, configuration, sensor_frame, stop_threads, vid, lidar_device,  car_location
    # initialization 
    init()
    # initialize the sensors
    lidar_device = RPLidar_Sensor(PORT_NAME='/dev/ttyUSB0',FOV=140,visualize=True)
    scan_data = [0]*360

    try:
        while(True):
            lidar_device.lidar.start()
            #for scan in lidar_device.lidar.iter_scans():
            scan = next(lidar_device.lidar.iter_scans()) 
            lidar_device.lidar.stop()   
            for (_, angle, distance) in scan:
                    scan_data[min([359, floor(angle)])] = distance
            lidar_d=lidar_device.process_lidar_data(scan_data)
            lidar_device.obj_coord_list=lidar_d
            print(f' {len(lidar_d)} objects -> {lidar_d}')
            # save lidar data
            save_lidar_data(lidar_d)           
            
            ## get camera data
            #save_image_data()
            
            ## get IMU data
            save_accelerometer_data()
            
            # get lidar data
            save_gps_data()
            
            # update the json file 
            save_json_file()
            
    except KeyboardInterrupt:
        lidar_device.strop_lidar() 
        sys.exit(0)
    except Exception as e:
        lidar_device.strop_lidar()
        print(f'\n\n {e}') 
        sys.exit(0)

def handle_ctrl_c(signal, frame):
    lidar_device = RPLidar_Sensor(PORT_NAME='/dev/ttyUSB0',FOV=140, visualize=False)
    print('Stoping.')
    lidar_device.lidar.stop()
    lidar_device.lidar.stop_motor()
    lidar_device.lidar.disconnect()
    print('Lidar stopped!!.')

    sys.exit(0)

signal.signal(signal.SIGINT, handle_ctrl_c)

if __name__ == "__main__":

    collect_node_data()
        
    
