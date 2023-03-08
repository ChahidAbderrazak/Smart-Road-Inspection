#!/usr/bin/env python
import os
import sys
import time
import cv2

###### ROS packages
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from lib.config_parameters import *
from lib.utils import *

# inputs
sensor_name="3D_CAMERA"                       
fs=10	                        # HZ							

# devces intilization
bridge = CvBridge()

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
 
def save_3D_camera_data(mission_dir, color_img, depth_img, sensor_frame):
    try:
        if color_img is None or depth_img is None :
            return 
        # save RGB/depth images
        RGB_filename_strg=timestamp=str(get_timestamp())+"_tmstmp_"+ str(get_sensor_filename(sensor_name, sensor_frame)) + ".jpg"
        RGB_image_save_path=os.path.join(mission_dir, "sweeps", sensor_name, RGB_filename_strg)
        depth_filename_strg=RGB_filename_strg[:-4] + "_depth.png"
        depth_image_save_path=os.path.join(mission_dir, "sweeps", sensor_name, depth_filename_strg)
        # print(sensor_name + ' data path=', RGB_image_save_path)

        # save RGB/Gray image 
        create_file_folder(RGB_image_save_path)
        cv2.imwrite(RGB_image_save_path, color_img)
        cv2.imwrite(depth_image_save_path, depth_img)
        # print('camera dict_frame:', dict_frame)
    except Exception as e:
        print('\n error: cannot save the published ROS data [ sensor= '+  sensor_name+ '] !') ; print(' Exception:', e)

def mission_callback(mission_msg):
    global mission_dir
    try:
        sensor_name="Mission folder"
        mission_dir = mission_msg.data
    except Exception as e:
        print('\n error: cannot save the published ROS data [ sensor= '+  sensor_name+ '] !') ; print(' Exception:', e)


###-------------------------  Subscribr to scheduler ------------------------###
mission_sub= rospy.Subscriber("/mission", String, mission_callback)

###-----------------------  3D CAMERA  -----------------------###
color_sub= rospy.Subscriber("/camera/color/image_raw", Image, color_callback)
depth_sub= rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_callback)
color_img, depth_img=None, None

rospy.init_node("s_"+sensor_name)
rate  = rospy.Rate(fs)
rospy.loginfo(" Topic= " + sensor_name)
# initialization    
idx=0
sensor_frame=0
start = time.time()
fs_sec='ND'
err=0
while not rospy.is_shutdown():
    try:
        # save the 3D camera images
        if mission_dir!='':
            save_3D_camera_data(mission_dir, color_img, depth_img, sensor_frame)
            sensor_frame+=1

        # rest scene
        rate.sleep()

        # timing the loop
        end = time.time()
        fs_sec=str(end - start)+ " sec"
        # print(sensor_name +' : fs_sec= ' + fs_sec )
        start = time.time()

        # display
        idx+=1
        if sensor_frame%100==1:
            msg=sensor_name + "[ data samples "+ str(sensor_frame) + "/"+ str(idx) + ", fs="+ str(fs) +\
                                 "sec ==> fs_op="+fs_sec + "]"
            rospy.loginfo(msg)
        
    except Exception as e:
        print(' Error in reading the ' +sensor_name +  ' sensor!!')
        print('Exception: ',e)
        err+=1
    except KeyboardInterrupt:
        break

    






