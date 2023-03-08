#!/usr/bin/env python
import os
import sys
import time
###### SENSORS packages
# Jetson Nano
from lib_jetson import SIM7600X_4G_sensors
from lib.utils import get_time_tag
# try:
# 	from lib_jetson import SIM7600X_4G_sensors
# except:
# 	from src.lib_jetson import SIM7600X_4G_sensors

###### ROS packages
import rospy
from std_msgs.msg import String

fs=10                            # HZ
sensor_name="GPS_sensor"
str_msg= String()
gps_pub= rospy.Publisher("/gps_location", String, queue_size=1)
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
        idx+=1
        # get GPS location
        err, dict_GPS= SIM7600X_4G_sensors.get_gps_data()
        car_location= dict_GPS["car_location"]
        # publish the position
        str_msg.data= str(dict_GPS)
        gps_pub.publish(str_msg)
        sensor_frame+=1
        
        # # rest scene
        # rate.sleep()

        # timing the loop
        end = time.time()
        fs_sec=str(end - start)+ " sec"
        start = time.time()
        #print('updating the position')

        # display
        if idx%10==1:
            msg=sensor_name + " data"+ str(idx) + "[ fs="+ str(fs) +\
                                 "sec ==> fs_op="+fs_sec + "]"
            rospy.loginfo(msg)
            print("GPS data "+ str(idx) + " : " ,  car_location)

    except Exception as e:
        print(' Error in reading the GPS sensor!!')
        print('Exception: ',e)

    except KeyboardInterrupt:
        break






