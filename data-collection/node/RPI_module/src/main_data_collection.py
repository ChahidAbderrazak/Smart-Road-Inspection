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
from lib_ahmad import HAIS_project

################################################## SENSOR FUNCTIONS #################################################
def save_lidar_data():
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame, scene_count, cam0, lidar_device, car_location
    ## LIDAR 
    scan_data=[0]*360
    scene_count+=1
    if lidar_device.lidar_connected:
        lidar_device.lidar.start()
        #for scan in lidar_device.lidar.iter_scans():
        # scan=next(lidar_device.lidar.iter_scans(scan_type='express', min_len=100))
        scan=next(lidar_device.lidar.iter_scans(max_buf_meas=2000))  
        scan=next(lidar_device.lidar.iter_scans())  
        lidar_device.lidar.stop()
        for (_, angle, distance) in scan:
                scan_data[min([359, floor(angle)])]=distance
        lidar_d=lidar_device.process_lidar_data(scan_data)
        lidar_device.obj_coord_list=lidar_d
        # save the colected data
        filename_strg=str(get_sensor_filename("LIDAR", sensor_frame)) + ".json"
        save_json_path=os.path.join(data_root, "sweeps", "LIDAR", filename_strg)
        save_json([{"points": lidar_d}], save_json_path)

        #if disp:
        #    print(f'\n - Lidar data={lidar_d}')
        # update outputs
        dict_frame=add_data_to_pipeline(sensor_frame)
        sensor_frame=sensor_frame + 1
        dict_frame["description"]=configuration["description"]
        dict_frame["timestamp"]=get_timestamp()
        dict_frame["scene"]=scene_count
        dict_frame["sensor_name"]="LIDAR"
        dict_frame["position"]={"Translation": car_location, 
                                "Rotation": []}
        dict_frame["calibration"]={"Translation": [], "Rotation": [], "Camera_intrinsic": ""}
        dict_frame["fileformat"]="json"
        dict_frame["filename"]=str(os.path.join("sweeps", "LIDAR", filename_strg))
        dict_frame["meta_data"]=""
def save_lidar_data_par():
    while True:
        save_lidar_data()

def save_image(sensor_name, frame):
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame, scene_count, car_location

    filename_strg=str(get_sensor_filename(sensor_name, sensor_frame)) + ".jpg"
    image_save_path=os.path.join(data_root, "sweeps", sensor_name, filename_strg)

    cv2.imwrite(image_save_path, frame)
    # update outputs
    dict_frame=add_data_to_pipeline(sensor_frame)
    sensor_frame=sensor_frame + 1
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

def save_road_camera_data():
    # save road camera file locally
    try:
        sensor_name="CSI_CAMERA"
        ret, frame=cam0.read()
        # save the sensor frame
        save_image(sensor_name, frame)
    except Exception as e:
        print(f'\n error: cannot read the {sensor_name} sensor! \n Exception: {e}')

    
def save_road_camera_data_par():
    while True:
        save_road_camera_data()


def save_lane_camera_data():
    # save left-sided camera
    try:
        sensor_name='LEFT_CAMERA' 
        ret, frame=cam_left.read()
        # save the sensor frame
        save_image(sensor_name, frame)
    except Exception as e:
        print(f'\n error: cannot read the {sensor_name} sensor! \n Exception: {e}')

   # save right-sided camera
    try:
        sensor_name='RIGHT_CAMERA'
        ret, frame=cam_right.read()
        # save the sensor frame
        save_image(sensor_name, frame)
    except Exception as e:
        print(f'\n error: cannot read the {sensor_name} sensor! \n Exception: {e}')


def save_lane_camera_data_par():
    while True:
        save_lane_camera_data()

def save_accelerometer_data():
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame, scene_count, car_location
    try:
        AccXangle, AccYangle, gyroXangle, gyroYangle, gyroZangle, CFangleX, CFangleY, heading, tiltCompensatedHeading, kalmanX, kalmanY=Imu.imuDt()
        #print("Accelerometer" + str(pygame.time.get_ticks()))
    except Exception as e:
        print(f'\n error: cannot read the IMU sensor! \n Exception: {e}')
        return 
    # update outputs
    dict_frame=add_data_to_pipeline(sensor_frame)
    sensor_frame=sensor_frame + 1
    dict_frame["description"]=configuration["description"]
    dict_frame["timestamp"]=get_timestamp()
    dict_frame["scene"]=scene_count
    dict_frame["sensor_name"]="IMU_SENSOR"
    dict_frame["position"]={"Translation": car_location, 
                              "Rotation": []}
    dict_frame["calibration"]={"Translation": [], "Rotation": [], "Camera_intrinsic": ""}
    dict_frame["fileformat"]=""
    dict_frame["filename"]=""
    dict_frame["meta_data"]={"ACCX Angle": AccXangle, "ACCY Angle": AccYangle, "GRYX Angle": gyroXangle, 
                               "GYRY Angle": gyroYangle, "GYRZ Angle": gyroZangle, "CFangleX Angle": CFangleX, 
                               "CFangleX Angle": CFangleY, "HEADING": heading, 
                               "tiltCompensatedHeading": tiltCompensatedHeading, "kalmanX": kalmanX, "kalmanY": kalmanY}

def save_gps_data():
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame, scene_count, car_location
    car_location=get_gps_data()
    #car_location=[lat, lng, alt]
    
    # update outputs
    dict_frame=add_data_to_pipeline(sensor_frame)
    sensor_frame=sensor_frame + 1
    dict_frame["description"]=configuration["description"]
    dict_frame["timestamp"]=get_timestamp()
    dict_frame["scene"]=scene_count
    dict_frame["sensor_name"]="GPS_SENSOR"
    dict_frame["position"]={"Translation": car_location, 
                              "Rotation": []}
    dict_frame["calibration"]={"Translation": [], "Rotation": [], "Camera_intrinsic": ""}
    dict_frame["fileformat"]=""
    dict_frame["filename"]=""
    dict_frame["meta_data"]=""

def save_mission_json_file():
    global dict_fr_list, filename
    if len(dict_fr_list)>100:    
        old_dict=load_json(filename)
        combined_dict=old_dict + dict_fr_list
        save_json(combined_dict, filename)
        dict_fr_list=[]
        if len(combined_dict)%1000==0:
            print(f"\n - Saving {str(len(combined_dict))}  frames in {filename}" )

def save_ego_data_par():
    while True:
        save_gps_data()
        save_accelerometer_data()
        save_mission_json_file()

def add_data_to_pipeline(frame):
    # function to add data to main sensor data dictionary based on corresponding frame
    global dict_fr_list, dict_frame
    if dict_frame == {}:
        dict_frame["frame"]=frame

    elif frame != dict_frame["frame"]:
        dict_fr_list.append(dict_frame)  # main list with all frames
        #print(dict_fr_list)
        dict_frame={}  # individual frame
        dict_frame["frame"]=frame
    return dict_frame

def uploading_to_firebase(th=1000000):
    cnt=-1
    while True:
        # display
        cnt+=1
        if cnt%5==0:
            print(f'\n - Uploading to Firebase [{cnt}] \n\t folder: {root}')
        
        # compress the data
        if HAIS_project.chek_the_data():
            HAIS_project.commpresss(th=th)

        # upload data to Firebase
        if os.listdir(root)!=[]:
            HAIS_project.send_data()

def init(msg):
    global car_location, sensor_frame, disp
    disp=False
    # display
    print(f'\n\n###################################')
    print(f'##   Running HAIS Datalogger [{msg}]')
    print(f'###################################\n\n')

    # itialize the variables
    dict_fr_list=[]
    # get the car position
    car_location= get_gps_data()
    #car_location= [-1, -1, -1]
    
    # define the log file
    data_log=load_json(filename)
    sensor_frame=1+len(data_log)
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
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame, scene_count, cam0, lidar_device, car_location
    # initialization 
    init('serial')
    # initialize the sensors
    lidar_device=RPLidar_Sensor(PORT_NAME=PORT_NAME, visualize=False)
    scene_count=0
    try:
        while(True):
            # start sensors recording
            scene_count+=1
            # LIDAR 
            save_lidar_data()
            # ROAD CAMERA
            save_road_camera_data()
            
            # IMU data
            save_accelerometer_data()
                
            # GPS data
            save_gps_data()
            
            # LANE CAMERA
            save_lane_camera_data()

            # update the mission file 
            save_mission_json_file()
    except KeyboardInterrupt:
        print(f'\n - Stopping Lidar after KeyboardInterrupt !!')
        lidar_device.stop_lidar() 
        sys.exit(0)

    except Exception as e:
        lidar_device.stop_lidar()
        print(f'\n\n stopping lidar after occured issue! \n Exception: {e}') 

def collect_node_data_par():
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame, scene_count, cam0, lidar_device, car_location
    # initialization 
    init('parallel')

    # initialize the sensors
    lidar_device=RPLidar_Sensor(PORT_NAME=PORT_NAME, visualize=False)
    # start sensors recording using threads for parallel processing
    lidar_thrd=threading.Thread(target=save_lidar_data_par)
    cam0_thrd=threading.Thread(target=save_road_camera_data_par)
    ego_thrd=threading.Thread(target=save_ego_data_par)
    lane_cam_thrd=threading.Thread(target=save_lane_camera_data_par)
    # upload_thrd=threading.Thread(target=uploading_to_firebase)

    # Start the treads
    lidar_thrd.start()
    cam0_thrd.start()
    ego_thrd.start()
    lane_cam_thrd.start()
    # upload_thrd.start()

if __name__ == "__main__":
    # collect_node_data()
    collect_node_data_par()
