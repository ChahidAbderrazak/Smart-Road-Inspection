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

str_msg= String()
str_pub= rospy.Publisher("/imu_data", String, queue_size=10)
# str_pub= rospy.Publisher("/gps_IMU", String, queue_size=10)
rospy.init_node("IMU_node")

# rate  = rospy.Rate(1) #1 Hz
# rate  = rospy.Rate(5) #5 Hz
idx=0
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
        if idx%100==1:
            rospy.loginfo("IMU data "+ str(idx) + " : " + str(IMU_dict))
        # rate.sleep()

    except Exception as e:
        print(' Error in reading the IMU sensor!!')
        print('Exception: ',e)
    except KeyboardInterrupt:
        break






