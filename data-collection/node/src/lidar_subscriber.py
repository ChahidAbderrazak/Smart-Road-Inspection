#!/usr/bin/env python
import os
import sys
import time
import cv2

###### ROS packages
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from lib.config_parameters import *
from lib.utils import *



# inputs
sensor_name="LIDAR"                    
fs=10	                        # HZ							

def scan_callback(scan_msg_data):
    global scan_ranges
    scan_ranges= np.asarray(scan_msg_data.ranges) #save scan as np array


def mission_callback(mission_msg):
    global mission_dir
    try:
        sensor_name="Mission folder"
        mission_dir = mission_msg.data
    except Exception as e:
        print('\n error: cannot save the published ROS data [ sensor= '+  sensor_name+ '] !') ; print(' Exception:', e)


###-------------------------  Subscribr to scheduler ------------------------###
mission_sub= rospy.Subscriber("/mission", String, mission_callback)

###-------------------------  LIDAR   ------------------------###
scan_sub= rospy.Subscriber("/scan", LaserScan, scan_callback)
scan_ranges=None

rospy.init_node(sensor_name)
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
        # save LIDAR sensor
        if mission_dir!='':
            save_lidar_data_v2 (scan_ranges, 
                                sensor_name=sensor_name,
                                sensor_frame=sensor_frame, 
                                mission_dir=mission_dir)
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
        if sensor_frame%10==1:
            msg=sensor_name + "[ data samples "+ str(sensor_frame) + "/"+ str(idx) + ", fs="+ str(fs) +\
                                 "sec ==> fs_op="+fs_sec + "]"
            rospy.loginfo(msg)
        
    except Exception as e:
        print(' Error in reading the ' +sensor_name +  ' sensor!!')
        print('Exception: ',e)
        err+=1
    except KeyboardInterrupt:
        break

    






