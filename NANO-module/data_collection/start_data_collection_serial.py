import os
import sys
import time
import signal
import threading
from lib.config_parameters import *
from lib.utils import *
from lib import Imu
from lib.Gps import *
from lib.lidar_sensor import *
# from rplidar import RPLidar
       
################################################## SENSOR FUNCTIONS #################################################
def save_lidar_data(lidar_d):
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame,scene_count, stop_threads, vid, lidar_device,  car_location

    filename_strg = str(get_sensor_filename("LIDAR", sensor_frame)) + ".json"
    save_json_path = os.path.join(data_root, "sweeps", "LIDAR", filename_strg)
    save_json([{"points": lidar_d}], save_json_path)

    #if disp:
    #    print(f'\n - Lidar data={lidar_d}')
    # update outputs
    dict_frame = add_data_to_pipeline(sensor_frame)
    sensor_frame = sensor_frame + 1
    dict_frame["description"] = configuration["description"]
    dict_frame["timestamp"] = get_timestamp()
    dict_frame["scene"] = scene_count
    dict_frame["sensor_name"] = "LIDAR"
    dict_frame["position"] = {"Translation": car_location,
                            "Rotation": []}
    dict_frame["calibration"] = {"Translation": [], "Rotation": [], "Camera_intrinsic": ""}
    dict_frame["fileformat"] = "json"
    dict_frame["filename"] = str(os.path.join("sweeps", "LIDAR", filename_strg))
    dict_frame["meta_data"] = ""
                
def save_mission_json_file():
    global dict_fr_list, filename
    if len(dict_fr_list)>5:    
        old_dict = load_json(filename)
        combined_dict = old_dict + dict_fr_list
        save_json(combined_dict, filename)
        dict_fr_list = []
        if len(combined_dict)%100==0:
            print(f"\n - Saving {str(len(combined_dict))}  frames in {filename}" )
        

def save_image_data():
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame,scene_count, stop_threads, vid,  car_location
    # save image file locally
    filename_strg = str(get_sensor_filename("CSI_CAMERA", sensor_frame)) + ".jpg"
    image_save_path = os.path.join(data_root, "sweeps", "CSI_CAMERA", filename_strg)
    ret, frame = vid.read()
    cv2.imwrite(image_save_path, frame)
    # update outputs
    dict_frame = add_data_to_pipeline(sensor_frame)
    sensor_frame = sensor_frame + 1
    dict_frame["description"] = configuration["description"]
    dict_frame["timestamp"] = get_timestamp()
    dict_frame["scene"] = scene_count
    dict_frame["sensor_name"] = "CSI_CAMERA"
    dict_frame["position"] = {"Translation": car_location,
                              "Rotation": []}
    dict_frame["calibration"] = {"Translation": [], "Rotation": [], "Camera_intrinsic": ""}
    dict_frame["fileformat"] = "jpg"
    dict_frame["filename"] = str(os.path.join("sweeps", "CSI_CAMERA", filename_strg))
    dict_frame["meta_data"] = ""

def save_accelerometer_data():
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame,scene_count, stop_threads, car_location

    AccXangle, AccYangle, gyroXangle, gyroYangle, gyroZangle, CFangleX, CFangleY, heading, tiltCompensatedHeading, kalmanX, kalmanY = Imu.imuDt()
    #print("Accelerometer" + str(pygame.time.get_ticks()))

    # update outputs
    dict_frame = add_data_to_pipeline(sensor_frame)
    sensor_frame = sensor_frame + 1
    dict_frame["description"] = configuration["description"]
    dict_frame["timestamp"] = get_timestamp()
    dict_frame["scene"] = scene_count
    dict_frame["sensor_name"] = "IMU_SENSOR"
    dict_frame["position"] = {"Translation": car_location,
                              "Rotation": []}
    dict_frame["calibration"] = {"Translation": [], "Rotation": [], "Camera_intrinsic": ""}
    dict_frame["fileformat"] = ""
    dict_frame["filename"] = ""
    dict_frame["meta_data"] = {"ACCX Angle": AccXangle, "ACCY Angle": AccYangle, "GRYX Angle": gyroXangle,
                               "GYRY Angle": gyroYangle, "GYRZ Angle": gyroZangle, "CFangleX Angle": CFangleX,
                               "CFangleX Angle": CFangleY, "HEADING": heading,
                               "tiltCompensatedHeading": tiltCompensatedHeading, "kalmanX": kalmanX, "kalmanY": kalmanY}

def save_gps_data():
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame,scene_count, stop_threads, car_location
    car_location = get_gps_data()
    #car_location=[lat, lng, alt]
    print(f"\n - car_location ={car_location}")
    
    # update outputs
    dict_frame = add_data_to_pipeline(sensor_frame)
    sensor_frame = sensor_frame + 1
    dict_frame["description"] = configuration["description"]
    dict_frame["timestamp"] = get_timestamp()
    dict_frame["scene"] = scene_count
    dict_frame["sensor_name"] = "GPS_SENSOR"
    dict_frame["position"] = {"Translation": car_location,
                              "Rotation": []}
    dict_frame["calibration"] = {"Translation": [], "Rotation": [], "Camera_intrinsic": ""}
    dict_frame["fileformat"] = ""
    dict_frame["filename"] = ""
    dict_frame["meta_data"] = ""

def add_data_to_pipeline(frame):
    # function to add data to main sensor data dictionary based on corresponding frame
    global dict_fr_list, dict_frame
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
    car_location= get_gps_data()
    #car_location= [-1,-1,-1]
    
    # define the log file
    data_log = load_json(filename)
    sensor_frame = 1+len(data_log)
    if sensor_frame==0:
        save_json([{}], filename)
    # set up folder to save data locally
    create_databse_folders(data_root)

    # create the mission info
    info_file_path=os.path.join(data_root, 'info.json')
    save_json(configuration, info_file_path)

    # display
    print(f"configuration={configuration}")
    print(f"filename={filename}")
    print(f"sensor_frame={sensor_frame}")

def collect_node_data():
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame,scene_count, stop_threads, vid, lidar_device,  car_location
    # initialization 
    init()
    # initialize the sensors
    lidar_device = RPLidar_Sensor(PORT_NAME='/dev/ttyUSB0',visualize=False)
    scan_data = [0]*360
    scene_count=0
    try:
        while(True):
            # start sensors recording
            scene_count+=1
            ## LIDAR 
            if lidar_device.lidar_connected:
                lidar_device.lidar.start()
                #for scan in lidar_device.lidar.iter_scans():
                scan = next(lidar_device.lidar.iter_scans())    
                for (_, angle, distance) in scan:
                        scan_data[min([359, floor(angle)])] = distance
                lidar_d=lidar_device.process_lidar_data(scan_data)
                lidar_device.obj_coord_list=lidar_d
                print(f' {len(lidar_d)} objects -> {lidar_d}')
                # save lidar data
                lidar_device.lidar.stop()
                save_lidar_data(lidar_d)           
            
            # CAMERA
            save_image_data()
            
            # IMU data
            try:
                save_accelerometer_data()
            except:
                print('\n error: cannot read the IMU sensor')
            
            # GPS data
            save_gps_data()
            
            # update the mission file 
            save_mission_json_file()
            
    except KeyboardInterrupt:
        lidar_device.stop_lidar() 
        sys.exit(0)
    except Exception as e:
        lidar_device.stop_lidar()
        print(f'\n\n {e}') 
        sys.exit(0)

if __name__ == "__main__":
    collect_node_data()
