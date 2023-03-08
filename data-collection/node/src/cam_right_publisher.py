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
sensor_name="RIGHT_CAMERA"
sensor_PORT=1                   # RIGHT Camera for lane marker capture
                           
cap= cv2.VideoCapture(sensor_PORT) 
# cap.set(cv2.CAP_PROP_FPS, fs)	
cam_fps = cap.get(cv2.CAP_PROP_FPS)	
fs=2*cam_fps	                        # HZ							
cam_right_pub= rospy.Publisher("/right_camera", String, queue_size=1)
rospy.init_node(sensor_name)
try:
    rate  = rospy.Rate(fs)
except: 
    fs=10
    rate  = rospy.Rate(fs)
rospy.loginfo(" Topic= " + sensor_name)
# devces intilization
bridge = CvBridge()

def mission_callback(mission_msg):
    global mission_dir
    try:
        sensor_name="Mission folder"
        mission_dir = mission_msg.data
    except Exception as e:
        print('\n error: cannot save the published ROS data [ sensor= '+  sensor_name+ '] !') ; print(' Exception:', e)


###-------------------------  Subscribr to scheduler ------------------------###
mission_sub= rospy.Subscriber("/mission", String, mission_callback)


# initialization    
idx=0
sensor_frame=0
start = time.time()
fs_sec='ND'
err=0
while not rospy.is_shutdown():
    try:
        # right-sided camera
        if cap.isOpened():
            err, rgb_frame1=save_cam_frame( cam=cap, 
                                            cam_pub=cam_right_pub, 
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

        else:
            err+=1
            rospy.loginfo(sensor_name + " cannot be opened. Please make sure it is connected")

            # # stop the ROS topic if the devoce is not connected
            # if err>50:
            #     break

        # display
        idx+=1
        if sensor_frame%50==1:
            msg="[Lane marker correct readings "+ str(sensor_frame) + "/"+ str(idx) + ", fs="+ str(fs) + "sec ==> fs_op="+fs_sec + ", cam fps="+ str(cam_fps)+ "]"
            rospy.loginfo(msg )
        
    except Exception as e:
        print(' Error in reading the ' +sensor_name +  ' sensor!!')
        print('Exception: ',e)

    except KeyboardInterrupt:
        break

    






