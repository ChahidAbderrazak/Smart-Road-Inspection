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
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge
import time
import numpy as np
############################################################################
fs=10                            # HZ

def init(msg):
    global data_root, mission_dir, mission_filename, sensor_frame, scene_count
    disp=False
    # display
    print('\n\n###################################')
    print('##   Running HAIS Datalogger [Jetson-' + msg +']')
    print('###################################\n\n')
    ###################### BUILDING THE DATABASE  ######################
    # create json file
    mission_name= "mission_" + get_time_tag(type=1)
    filename_strg, _ = get_file_names(configuration)
    data_root=os.path.join(data_root, mission_name)
    mission_dir=data_root
    mission_filename = os.path.join(data_root, "log", filename_strg)
    mission_folder=os.path.dirname(mission_filename)
    create_new_folder(mission_folder)
    # set up folder to save data locally
    create_databse_folders(data_root)
    # define the log file
    data_log = load_json(mission_filename)
    sensor_frame = 1+len(data_log)
    scene_count=0
    new_data_log=[]

    # create the mission info
    # if sensor_frame==0:
    #     save_json([{}], mission_filename)
    # create the mission info
    info_file_path=os.path.join(data_root, 'info.json')
    save_json(configuration, info_file_path)

    # display
    print("configuration=", configuration)
    print("filename=", mission_filename)
    print("sensor_frame=", sensor_frame)
    print("mission file folder=", data_root)

def scan_callback(scan_msg_data):
    global scan_ranges
    scan_ranges= np.asarray(scan_msg_data.ranges) #save scan as np array

def gps_callback(str_msg):
    global dict_GPS
    try:
        sensor_name="GPS_SENSOR"
        dict_GPS = ast.literal_eval(str_msg.data) 
        # print(' published GPS ', dict_GPS )
    except Exception as e:
        print('\n error: cannot save the published ROS data [ sensor= '+  sensor_name+ '] !') ; print(' Exception:', e)

def save_gps_data(sensor_frame):
    global dict_GPS, dict_frame, dict_fr_list, sensor_name
    if dict_GPS!={}:
        try:
            sensor_name="GPS_SENSOR"
            # update mission file
            dict_frame, dict_fr_list=add_data_to_pipeline(sensor_frame)
            # print("\n flag:  dict_GPS=", dict_GPS)
            car_location= dict_GPS["car_location"]
            Speed= dict_GPS["Speed"]
            Navigation_Angle= dict_GPS["Navigation_Angle"]       
            # save sensor data
            dict_frame=save_GPS_file_v2(sensor_name=sensor_name, 
                                        sensor_frame=sensor_frame,
                                        car_location=car_location, 
                                        dict_GPS=dict_GPS)
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

def save_imu_data(sensor_frame):
    global IMU_dict, dict_frame, dict_fr_list, sensor_name
    if IMU_dict!={}:
        try:
            sensor_name="IMU_SENSOR"
            # update mission file
            dict_frame, dict_fr_list=add_data_to_pipeline(sensor_frame)
            
            # save sensor data
            dict_frame=save_IMU_file_v2(sensor_name=sensor_name, 
                                        sensor_frame=sensor_frame, 
                                        IMU_data=IMU_dict)

        except Exception as e:
            print('\n error: waiting for the '+  sensor_name+ ' sensor RoS data!') ; print(' Exception:', e)

def add_data_to_pipeline(sensor_frame):
    global dict_frame, dict_fr_list, scene_count

    # function to add data to main sensor data dictionary based on corresponding frame
    # print("\n flag:  dict_frame=", dict_frame)
    # print("\n flag:  sensor_frame=", sensor_frame)
    # print("\n ")
    if dict_frame == {}:
        dict_frame["frame"] = sensor_frame
        scene_count=0

    elif sensor_frame != dict_frame["frame"]:
        if not dict_frame in dict_fr_list:
            dict_fr_list.append(dict_frame)  # main list with all frames
        # print(dict_fr_list)
        dict_frame = {}  # individual frame
        dict_frame["frame"] = sensor_frame

    return dict_frame, dict_fr_list


def save_mission_json_file(dict_fr_list, mission_filename ):
    # display 
    if os.path.exists(mission_filename):
        old_dict = load_json(mission_filename)
    else:
        old_dict=[]

    # combine the files
    dict_fr_list=list(np.unique(dict_fr_list))
    dict_fr_list=[val for val in dict_fr_list if val!={}] 
    combined_dict = old_dict + dict_fr_list
    save_json(combined_dict, mission_filename)
    dict_fr_list = []
    if len(combined_dict)%10==0:
        print("\n - Saving " + str(len(combined_dict))+" frames in "+str(mission_filename) )

def update_pipeline():
    global mission_filename, dict_fr_list
    # sensor_filename=mission_filename[:-4]+sensor_name+'.json'
    save_mission_json_file(dict_fr_list, mission_filename )

    # # display
    # print('')
    # print( '| ', sensor_name )
    # print('dict_frame=', dict_frame)
    # print('dict_fr_list [ size=', len(dict_fr_list),']')
    # print('dict_fr_list [ size=', len(dict_fr_list),']=', dict_fr_list)

#----------------- define publishers --------------------
# sample_dict_pub= rospy.Publisher("/sample_dict", String, queue_size=10)
mission_pub= rospy.Publisher("/mission", String, queue_size=1)

###-------------------------  GPS   ------------------------###
dict_GPS={}
gps_sub= rospy.Subscriber("/gps_location", String, gps_callback)

###-------------------------  IMU   ------------------------###
imu_sub= rospy.Subscriber("/imu_data", String, imu_callback)
IMU_dict={}
# intialization 
node_name= "HAIS_JN_"+configuration["vehicle"]
rospy.init_node(node_name) 
rospy.loginfo(" Topic= Datalogger" )
rate  = rospy.Rate(fs) 

###-------------------------  LED Signalization   ------------------------###
import Jetson.GPIO as GPIO
GPIO.setmode(GPIO.BOARD) 
# Pin Definition
red_pin = 11
yellow_pin = 12

no_internet_LED_flag=False
yellow_LED_flag=True
# setup the I/O
GPIO.setup(yellow_pin, GPIO.OUT, initial=GPIO.HIGH) 
GPIO.setup(red_pin, GPIO.OUT, initial=GPIO.HIGH) 
#---------------------------------------------------------------------------

def scanning_signalization_LED(yellow_LED_flag):
    if yellow_LED_flag:
        GPIO.output(yellow_pin, GPIO.HIGH) 
        # print("Yellow LED is ON")
    else:
        GPIO.output(yellow_pin, GPIO.LOW)
        # print("Yellow LED is OFF")

def internet_signalization_LED(no_internet_LED_flag):
    if no_internet_LED_flag:
        GPIO.output(red_pin, GPIO.HIGH) 
        print("No internet connection !!!")
    else:
        GPIO.output(red_pin, GPIO.LOW)
        # print(" Internet connection established")

def ros_run_data_collection():
    global mission_dir, mission_filename, scene_count, no_internet_LED_flag, yellow_LED_flag, right_cam_frame, left_cam_frame

    # initialization
    init('Serial')

    #Start listening for subscriptions without halting the program flow
    t1= threading.Thread(target=rospy.spin) #thread for rospy.spin
    t1.start() 
    rospy.loginfo('HAIS Node [' +node_name + '] Initialized '+str(fs)+'Hz')
    
    # start the data collection
    idx=0
    sensor_frame=0       
    while not rospy.is_shutdown():
        idx+=1
        
        # publish the mission file folder path
        mission_pub.publish(mission_dir)
        
        # # save GPS sensor
        save_gps_data(sensor_frame)
        update_pipeline()
        sensor_frame+=1
        
        # # save IMU sensor
        save_imu_data(sensor_frame)
        update_pipeline()
        sensor_frame+=1
        
        # signalization
        scanning_signalization_LED(yellow_LED_flag)
        internet_signalization_LED(no_internet_LED_flag)

        # switch the yellow LED
        yellow_LED_flag= not yellow_LED_flag 
        # no_internet_LED_flag= not no_internet_LED_flag 

        # rest scene
        rate.sleep()
        
        # monitoring the progress
        if idx%1000==0:
            print('\n\n')
            rospy.loginfo("Collected: "+ str(idx) + " samples ")
        if idx%100==1:
            rospy.loginfo("Datalogger data: "+ str(idx) + " samples" )
    ## terminate thread
    t1.join()

    # switch off signalization LEDS 
    scanning_signalization_LED(False)
    internet_signalization_LED(False)

if __name__ == "__main__":
    ros_run_data_collection()