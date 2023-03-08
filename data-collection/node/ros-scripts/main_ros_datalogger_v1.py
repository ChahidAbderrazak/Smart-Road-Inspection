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
############################################################################
# devices intilization
bridge = CvBridge()  ## 3D camera 
bridge2 = CvBridge() ## LANE MARKER camera 
def init(msg):
    global mission_filename, sensor_frame, scene_count
    disp=False
    # display
    print('\n\n###################################')
    print('##   Running HAIS Datalogger [Jetson-' + msg +']')
    print('###################################\n\n')
    ###################### BUILDING THE DATABASE  ######################
    # create json file
    filename_strg, _ = get_file_names(configuration)
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
    print("data_root=", data_root)

def save_road_camera_data():
    global data_root, dict_fr_list, dict_frame, configuration, car_location, sensor_frame, dict_fr_list, scene_count
    # save road camera file locally
    try:
        sensor_name="CSI_CAMERA"
        # capture the cam frame
        ret, frame=CSI_cam.read()
        if frame is None:
            return 
        # update mission file
        dict_frame, dict_fr_list=add_data_to_pipeline(sensor_frame)

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
    global dict_frame, dict_fr_list, sensor_name, scene_count, sensor_frame
    sensor_name="3D_CAMERA"
    try:
        if color_img is None or depth_img is None :
            return 
        # update mission file
        dict_frame, dict_fr_list=add_data_to_pipeline(sensor_frame)
        sensor_frame+=1
        # save RGB/depth images
        dict_frame=save_3D_image(sensor_name=sensor_name, rgb_frame=color_img, depth_frame=depth_img, sensor_frame=sensor_frame)
        sensor_frame+=1
        # print('camera dict_frame:', dict_frame)
    except Exception as e:
        print('\n error: cannot save the published ROS data [ sensor= '+  sensor_name+ '] !') ; print(' Exception:', e)

def right_camera_callback(msg_right):
    global right_cam_frame
    right_cam_frame = bridge2.imgmsg_to_cv2(msg_right, desired_encoding='passthrough')

def save_right_camera_data(right_cam_frame):
    global dict_frame, dict_fr_list, sensor_name, scene_count, sensor_frame
    sensor_name="RIGHT_CAMERA"
    try:
        if right_cam_frame is None:
            print('- empty '+ sensor_name + ' :' , right_cam_frame)

            return 
        # update mission file
        dict_frame, dict_fr_list=add_data_to_pipeline(sensor_frame)
        sensor_frame+=1
        # save RGB image
        dict_frame=save_image(sensor_name=sensor_name, frame=right_cam_frame, sensor_frame=sensor_frame)
        sensor_frame+=1
    except Exception as e:
        print('\n error: cannot save the published ROS data [ sensor= '+  sensor_name+ '] !') ; print(' Exception:', e)

def left_camera_callback(msg_left): 
    global left_cam_frame   
    left_cam_frame = bridge2.imgmsg_to_cv2(msg_left, desired_encoding='passthrough')

def save_left_camera_data(left_cam_frame):
    global dict_frame, dict_fr_list, sensor_name, scene_count, sensor_frame
    sensor_name="LEFT_CAMERA"
    try:
        if left_cam_frame is None:
            print('- empty '+ sensor_name + ' :' , left_cam_frame)
            return 
        # update mission file
        dict_frame, dict_fr_list=add_data_to_pipeline(sensor_frame)
        sensor_frame+=1
        # save RGB image
        dict_frame=save_image(sensor_name=sensor_name, frame=left_cam_frame, sensor_frame=sensor_frame)
        sensor_frame+=1
    except Exception as e:
        print('\n error: cannot save the published ROS data [ sensor= '+  sensor_name+ '] !') ; print(' Exception:', e)

def scan_callback(scan_msg_data):
    global scan_ranges
    scan_ranges= np.asarray(scan_msg_data.ranges) #save scan as np array

def save_lidar_data(scan_ranges):
    global dict_frame, dict_fr_list, sensor_name, sensor_frame
    try:
        sensor_name="LIDAR"
        # update mission file
        dict_frame, dict_fr_list=add_data_to_pipeline(sensor_frame)
        # save lidar data
        dict_frame=save_lidar_file(sensor_name, scan_ranges ,sensor_frame)
        sensor_frame+=1
    except Exception as e:
        print('\n error: cannot save the published ROS data [ sensor= '+  sensor_name+ '] !') ; print(' Exception:', e)

def gps_callback(str_msg):
    global dict_GPS
    try:
        sensor_name="GPS_SENSOR"
        dict_GPS = ast.literal_eval(str_msg.data) 
        # print(' published GPS ', dict_GPS )
    except Exception as e:
        print('\n error: cannot save the published ROS data [ sensor= '+  sensor_name+ '] !') ; print(' Exception:', e)

def save_gps_data():
    global dict_GPS, dict_frame, dict_fr_list, sensor_name, sensor_frame
    try:
        # update mission file
        dict_frame, dict_fr_list=add_data_to_pipeline(sensor_frame)
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

def save_imu_data():
    global IMU_dict, dict_frame, dict_fr_list, sensor_name, sensor_frame

    try:
        sensor_name="IMU_SENSOR"
        # update mission file
        dict_frame, dict_fr_list=add_data_to_pipeline(sensor_frame)
        
        # save sensor data
        dict_frame=save_IMU_file(sensor_name, IMU_dict)
        sensor_frame+=1

    except Exception as e:
        print('\n error: waiting for the '+  sensor_name+ ' sensor RoS data!') ; print(' Exception:', e)

def add_data_to_pipeline(frame):
    # global car_location, sensor_frame, dict_frame, dict_fr_list, scene_count
    global dict_frame, dict_fr_list, scene_count

    # function to add data to main sensor data dictionary based on corresponding frame

    if dict_frame == {}:
        dict_frame["frame"] = frame
        scene_count=0

    elif frame != dict_frame["frame"]:
        if not dict_frame in dict_fr_list:
            dict_fr_list.append(dict_frame)  # main list with all frames
        # print(dict_fr_list)
        dict_frame = {}  # individual frame
        dict_frame["frame"] = frame

    return dict_frame, dict_fr_list

def save_mission_json_file0():
    global mission_filename , Speed, Navigation_Angle, sensor_frame, dict_frame, sensor_name, dict_frame, dict_fr_list

    print('------------------------------------------------------')
    print('------------------------------------------------------')

    if len(dict_fr_list)>4:    
        # display 
        old_dict = load_json(mission_filename)
        combined_dict = old_dict + dict_fr_list
        save_json(combined_dict, mission_filename)
        dict_fr_list = []
        if len(combined_dict)%10==0:
            print("\n - Saving " + str(len(combined_dict))+" frames in "+str(mission_filename) )
    else:
        print('dict_fr_list [ size=', len(dict_fr_list),']=', dict_fr_list)
        # sys.exit(0)

def save_mission_json_file(dict_fr_list, mission_filename ):

    # if len(dict_fr_list)>4:    
    # display 
    if os.path.exists(mission_filename):
        old_dict = load_json(mission_filename)
    else:
        old_dict=[]

    # combine the files
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


#----------------- define the sensors subscriber --------------------

###-----------------------  3D CAMERA  -----------------------###
color_sub= rospy.Subscriber("/camera/color/image_raw", Image, color_callback)
depth_sub= rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_callback)
color_img, depth_img=None, None
###---------------------  SIDE camera   ----------------------###
cam_right_sub= rospy.Subscriber("/right_camera", Image, right_camera_callback)
cam_left_sub= rospy.Subscriber("/left_camera", Image, left_camera_callback)
right_cam_frame,left_cam_frame=None,None

###-------------------------  LIDAR   ------------------------###
scan_sub= rospy.Subscriber("/scan", LaserScan, scan_callback)
scan_ranges=None
###-------------------------  GPS   ------------------------###
gps_sub= rospy.Subscriber("/gps_location", String, gps_callback)
dict_GPS={}
###-------------------------  IMU   ------------------------###
imu_sub= rospy.Subscriber("/imu_data", String, imu_callback)
IMU_dict={}

# intialization 
node_name= "HAIS_JN_"+configuration["vehicle"]
rospy.init_node(node_name) 
rate  = rospy.Rate(60) # HZ

###-------------------------  LED Signalization   ------------------------###
import Jetson.GPIO as GPIO
GPIO.setmode(GPIO.BOARD) 
# Pin Definition
red_pin = 11
yellow_pin = 12

no_internet_LED_flag=True
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
    global scene_count, no_internet_LED_flag, yellow_LED_flag, right_cam_frame, left_cam_frame

    # initialization
    init('Serial')

    #Start listening for subscriptions without halting the program flow
    t1= threading.Thread(target=rospy.spin) #thread for rospy.spin
    t1.start() 
    rospy.loginfo('HAIS Node [' +node_name + '] Initialized '+str(fs)+'Hz')
    idx=0
    while not rospy.is_shutdown():
        idx+=1
        # rate.sleep()

        # save 3D camera: RGB/depth images
        save_3D_camera_data(color_img, depth_img)
        update_pipeline()
        
        # save RIGHT cameras: RGB
        save_right_camera_data(right_cam_frame)
        update_pipeline()

        # save LEFT cameras: RGB
        save_left_camera_data(left_cam_frame)
        update_pipeline()

        # save LIDAR sensor
        save_lidar_data(scan_ranges)
        update_pipeline()
        
        # # save IMU sensor
        save_imu_data()
        update_pipeline()

        # signalization
        scanning_signalization_LED(yellow_LED_flag)
        internet_signalization_LED(no_internet_LED_flag)

        # switch the yellow LED
        yellow_LED_flag= not yellow_LED_flag 
        # no_internet_LED_flag= not no_internet_LED_flag 

        # rest scene
        scene_count+=1
        rate.sleep()

        # if idx==4:
        #     print('dict_fr_list [ size=', len(dict_fr_list),']=', dict_fr_list)
        #     break
        
        # monitoring the progress
        if idx%100==0:
            print('\n\n')
            rospy.loginfo("Node collected: "+ str(idx) + " samples ")
        if idx%10==1:
            rospy.loginfo("Node collected: "+ str(idx) + " samples ")
        
    ## terminate thread
    t1.join()

    # switch off signalization LEDS 
    scanning_signalization_LED(False)
    internet_signalization_LED(False)

if __name__ == "__main__":
    ros_run_data_collection()