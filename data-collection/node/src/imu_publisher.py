#!/usr/bin/env python
import os
import sys
import time

###### SENSORS packages
# Jetson Nano
try:
	from lib_jetson import ENV_IMU_sensors
except:
	from src.lib_jetson import ENV_IMU_sensors

###### ROS packages
import rospy
from std_msgs.msg import String

fs=10      #Hz
sensor_name="IMU_sensor"

str_msg= String()
str_pub= rospy.Publisher("/imu_data", String, queue_size=10)
# str_pub= rospy.Publisher("/gps_IMU", String, queue_size=10)
rospy.init_node("IMU_node")
rate  = rospy.Rate(fs)
rospy.loginfo(" Topic= " + sensor_name)
idx=0
sensor_frame=0
while not rospy.is_shutdown():
    idx+=1
    try:
        dict_Kinetic=ENV_IMU_sensors.get_IMU_data(LCD_visualize=False)
        dict_wheather=ENV_IMU_sensors.get_wheather_sensors_data(LCD_visualize=False)

        IMU_dict={}
        IMU_dict.update(dict_Kinetic)
        IMU_dict.update(dict_wheather)
        # print('\n - car_location =', car_location)
        str_msg.data= str(IMU_dict)
        str_pub.publish(str_msg)

        sensor_frame+=1
         # display
        idx+=1
        if sensor_frame%10==1:
            msg=sensor_name + "[ data samples "+ str(sensor_frame) + "/"+ str(idx) + ", fs="+ str(fs) +\
                                 "sec ==> fs_op="+fs_sec + "]"
            rospy.loginfo(msg)
        

    except Exception as e:
        print(' Error in reading the IMU sensor!!')
        print('Exception: ',e)
    except KeyboardInterrupt:
        break






