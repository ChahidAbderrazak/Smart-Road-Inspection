#!/usr/bin/env python
import os
import sys
import time
import ast
import signal
import threading

###### Sensors packages
from lib.config_parameters import *
from lib.utils import *
# from lib_RPI import Imu
# from lib_RPI.Gps import *
# from lib_RPI.lidar_sensor import *
# from lib_ahmad import HAIS_project


###### ROS packages
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge
import time
import numpy as np

# devces intilization
bridge = CvBridge()  ## 3D camera 
CSI_cam = cv2.VideoCapture(1)	

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
        print('\n error: cannot save the '+  sensor_name+ ' sensor!') ; print(' Exception:', e)
    

def color_callback(color_msg_data):
    global color_img
    # get GPS 3D Camera RGB image
    color_img = bridge.imgmsg_to_cv2(color_msg_data, desired_encoding='passthrough') 

def depth_callback(depth_msg_data):
    global depth_img
    # get/save 3D Camera Depth image
    depth_img = bridge.imgmsg_to_cv2(depth_msg_data, desired_encoding='passthrough')
 

def save_3D_camera_data(color_img, depth_img):
    global data_root, dict_fr_list, dict_frame, configuration, car_location, sensor_frame, dict_fr_list, scene_count
    try:
        sensor_name="3D_CAMERA"
        # update mission file
        dict_frame=add_data_to_pipeline(sensor_frame)
        sensor_frame+=1
        # save RGB image
        dict_frame=save_image(sensor_name=sensor_name, frame=color_img, sensor_frame=sensor_frame)
        sensor_frame+=1
        # save depth image
        dict_frame=add_data_to_pipeline(sensor_frame)
        dict_frame=save_image(sensor_name=sensor_name, frame=depth_img, sensor_frame=sensor_frame, tag='depth')
        sensor_frame+=1
    except Exception as e:
        print('\n error: cannot save the published ROS data [ sensor= '+  sensor_name+ '] !') ; print(' Exception:', e)

def scan_callback(scan_msg_data):
    global scan_ranges
    scan_ranges= np.asarray(scan_msg_data.ranges) #save scan as np array

def save_lidar_data(scan_ranges):
    global car_location, sensor_frame, dict_fr_list
    try:
        sensor_name="LIDAR"
        # update mission file
        dict_frame=add_data_to_pipeline(sensor_frame)
        # save lidar data
        dict_frame=save_lidar_file(sensor_name, scan_ranges ,sensor_frame)
        sensor_frame+=1
    except Exception as e:
        print('\n error: cannot save the published ROS data [ sensor= '+  sensor_name+ '] !') ; print(' Exception:', e)

def gps_callback(str_msg):
    global car_location, Speed, Navigation_Angle, sensor_frame, dict_frame
    try:
        sensor_name="GPS_SENSOR"
        dict_GPS = ast.literal_eval(str_msg.data) 
        # print(' published GPS ', dict_GPS )
       
        # update mission file
        dict_frame=add_data_to_pipeline(sensor_frame)
        car_location= dict_GPS["car_location"]
        Speed= dict_GPS["Speed"]
        Navigation_Angle= dict_GPS["Navigation_Angle"]       
        # save sensor data
        dict_frame=save_GPS_file(sensor_name, car_location=car_location, dict_GPS=dict_GPS)
        sensor_frame+=1
    except Exception as e:
        print('\n error: cannot save the published ROS data [ sensor= '+  sensor_name+ '] !') ; print(' Exception:', e)

def imu_callback(str_msg):
    global IMU_dict
    try:
        sensor_name="IMU_SENSOR"
        # retreive the ROS-published data
        IMU_dict = ast.literal_eval(str_msg.data) 
        # print(' published IMU ', IMU_dict )
    except Exception as e:
        print('\n error: cannot save the published ROS data [ sensor= '+  sensor_name+ '] !') ; print(' Exception:', e)
        sys.exit(0)

def save_imu_data():
    global IMU_dict, car_location, sensor_frame, dict_fr_list, dict_frame
    try:
        sensor_name="IMU_SENSOR"
        # update mission file
        dict_frame=add_data_to_pipeline(sensor_frame)
        
        # save sensor data
        dict_frame=save_IMU_file(sensor_name, IMU_dict)
        sensor_frame+=1

    except Exception as e:
        print('\n error: waiting for the '+  sensor_name+ ' sensor RoS data!') ; print(' Exception:', e)


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
    if len(dict_fr_list)>48:    
        # display 
        old_dict = load_json(mission_filename)
        combined_dict = old_dict + dict_fr_list
        save_json(combined_dict, mission_filename)
        dict_fr_list = []
        if len(combined_dict)%10==0:
            print("\n - Saving " + str(len(combined_dict))+" frames in "+str(mission_filename) )
               
def init(msg):
    global data_root, mission_filename, car_location, sensor_frame, disp
    disp=False
    # display
    print('\n\n###################################')
    print('##   Running HAIS Datalogger [Jetson-' + msg +']')
    print('###################################\n\n')

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
    
############################################################################
# create the database folders 
init('Serial')
#define the sensors subscriber
###-----------------------  3D CAMERA  -----------------------###
color_sub= rospy.Subscriber("/camera/color/image_raw", Image, color_callback)
depth_sub= rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_callback)
###-------------------------  LIDAR   ------------------------###
scan_sub= rospy.Subscriber("/scan", LaserScan, scan_callback)
###-------------------------  GPS   ------------------------###
gps_sub= rospy.Subscriber("/gps_location", String, gps_callback)
dict_GPS={}
###-------------------------  IMU   ------------------------###
imu_sub= rospy.Subscriber("/imu_data", String, imu_callback)
IMU_dict={}

# intialization 
node_name= "HAIS_Jetson_node_"+configuration["vehicle"]
rospy.init_node(node_name) 
# rate_slow = rospy.Rate(fs)


#Start listening for subscriptions without halting the program flow
t1= threading.Thread(target=rospy.spin) #thread for rospy.spin
t1.start() 
rospy.loginfo('HAIS Node [' +node_name + '] Initialized '+str(fs)+'Hz')
while not rospy.is_shutdown():
    try:
        # save CSI camera sensor
        save_road_camera_data()

        # # save LIDAR sensor
        # save_lidar_data(scan_ranges)

        # save 3D camea: RGB/depth images
        save_3D_camera_data(color_img, depth_img)

        # save IMU sensor
        save_imu_data()

        # save mission file
        save_mission_json_file()

    except Exception as e:
        print('\n error in the datalogger loop!') ; print(' Exception:', e)
    except KeyboardInterrupt:
        break

## terminate thread
t1.join()

