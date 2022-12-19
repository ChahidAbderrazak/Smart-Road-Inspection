import os
import sys
import time
import signal
import threading
from lib.config_parameters import *
from lib.utils import *
from lib_RPI import Imu
from lib_RPI.Gps import *
from lib_RPI.lidar_sensor import *

##########  camera 
CSI_cam = cv2.VideoCapture(0)													# Main Camera for the road
cam_right= cv2.VideoCapture(1)												# RIGHT Camera for the right lane marker
cam_left= cv2.VideoCapture(2)												# LEFT Camera for the left lane marker
################################################## SENSOR FUNCTIONS #################################################
def save_lidar_data():
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame, scene_count, lidar_device, car_location
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

        # update mission file
        sensor_name="LIDAR"
        dict_frame=add_data_to_pipeline(sensor_frame)
        # save the colected data
        save_lidar_file(sensor_name=sensor_name, lidar_d=lidar_d ,sensor_frame=sensor_frame)
        sensor_frame+=1

def save_lidar_data_par():
    while True:
        save_lidar_data()

def save_road_camera_data():
    global data_root, dict_fr_list, dict_frame, configuration, car_location, sensor_frame, dict_fr_list, scene_count

    # save road camera file locally
    try:
        sensor_name="CSI_CAMERA"
        # capture the cam frame
        ret, frame=CSI_cam.read()
        # update mission file
        dict_frame=add_data_to_pipeline(sensor_frame)

        # save RGB image
        dict_frame=save_image(sensor_name=sensor_name, frame=frame, sensor_frame=sensor_frame)
        sensor_frame+=1
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
        IMU_data=Imu.imuDt()
        # update outputs
        sensor_name="IMU_SENSOR"
        save_IMU_file(sensor_name, IMU_data)
    except Exception as e:
        print(f'\n error: cannot read the IMU sensor! \n Exception: {e}')
        return 
    
def save_gps_data():
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame, scene_count, car_location
    car_location=get_gps_data()
    #car_location=[lat, lng, alt]
    
    # update outputs
    sensor_name="GPS_SENSOR"
    save_GPS_file(sensor_name, car_location)

def save_ego_data_par():
    while True:
        save_gps_data()
        save_accelerometer_data()
        save_mission_json_file()

def add_data_to_pipeline(frame):
    global car_location, sensor_frame, dict_frame, dict_fr_list, scene_count
    # function to add data to main sensor data dictionary based on corresponding frame
    if dict_frame == {}:
        dict_frame["frame"] = frame
        scene_count=0

    elif frame != dict_frame["frame"]:
        dict_fr_list.append(dict_frame)  # main list with all frames
        #print(dict_fr_list)
        dict_frame = {}  # individual frame
        dict_frame["frame"] = frame

    return dict_frame
def save_mission_json_file():
    global dict_fr_list, mission_filename
    if len(dict_fr_list)>100:    
        old_dict=load_json(mission_filename)
        combined_dict=old_dict + dict_fr_list
        save_json(combined_dict, mission_filename)
        dict_fr_list=[]
        if len(combined_dict)%1000==0:
            print(f"\n - Saving {str(len(combined_dict))}  frames in {filename}" )

# def uploading_to_firebase(th=1000000):
#     cnt=-1
#     while True:
#         # display
#         cnt+=1
#         if cnt%5==0:
#             print(f'\n - Uploading to Firebase [{cnt}] \n\t folder: {tmp_folder}')
        
#         # compress the data
#         if HAIS_project.chek_the_data():
#             HAIS_project.commpresss(th=th)

#         # upload data to Firebase
#         if os.listdir(data_root)!=[]:
#             HAIS_project.send_data()

def init(msg):
    global data_root, mission_filename, car_location, sensor_frame, disp
    disp=False
    # display
    print(f'\n\n###################################')
    print(f'##   Running HAIS Datalogger [RPi-{msg}]')
    print(f'###################################\n\n')

    # intialize global variables
    car_location=get_gps_data() #[-1,-1,-1]										# intial car position (waiting for the GPS to start)

    ###################### BUILDING THE DATABASE  ######################
    # create json file
    filename_strg, _ = get_file_names(configuration)
    mission_filename = os.path.join(data_root, "missions", filename_strg)
    mission_folder=os.path.dirname(mission_filename)
    create_new_folder(mission_folder)
    # set up folder to save data locally
    create_databse_folders(data_root)
    # define the log file
    data_log = load_json(mission_filename)
    sensor_frame = 1+len(data_log)
    # if sensor_frame==0:
    #     save_json([{}], mission_filename)
    # create the mission info
    info_file_path=os.path.join(data_root, 'info.json')
    save_json(configuration, info_file_path)

    # display
    print("configuration=", configuration)
    print("filename=", mission_filename)
    print("sensor_frame=", sensor_frame)
    print("data_root=", data_root)

def collect_node_data():
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame, scene_count, lidar_device, car_location
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
    global data_root, dict_fr_list, dict_frame, configuration, sensor_frame, scene_count, lidar_device, car_location
    # initialization 
    init('parallel')

    # initialize the sensors
    lidar_device=RPLidar_Sensor(PORT_NAME=PORT_NAME, visualize=False)
    # start sensors recording using threads for parallel processing
    lidar_thrd=threading.Thread(target=save_lidar_data_par)
    cam0_thrd=threading.Thread(target=save_road_camera_data_par)
    ego_thrd=threading.Thread(target=save_ego_data_par)
    # lane_cam_thrd=threading.Thread(target=save_lane_camera_data_par)
    # upload_thrd=threading.Thread(target=uploading_to_firebase)

    # Start the treads
    lidar_thrd.start()
    cam0_thrd.start()
    ego_thrd.start()
    # lane_cam_thrd.start()
    # upload_thrd.start()

if __name__ == "__main__":
    # collect_node_data()
    collect_node_data_par()
