#!/usr/bin/env python
import os
import sys
import time

###### SENSORS packages
# Jetson Nano
from lib_jetson import SIM7600X_4G_sensors
# try:
# 	from lib_jetson import SIM7600X_4G_sensors
# except:
# 	from src.lib_jetson import SIM7600X_4G_sensors

###### ROS packages
import rospy
from std_msgs.msg import String

str_msg= String()
str_pub= rospy.Publisher("/gps_location", String, queue_size=10)
rospy.init_node("GPS_node")

# rate  = rospy.Rate(1) #1 Hz
# rate  = rospy.Rate(5) #5 Hz
idx=0
while not rospy.is_shutdown():
    idx+=1
    try:
        dict_GPS=SIM7600X_4G_sensors.get_gps_data()
        str_msg.data= str(dict_GPS)
        str_pub.publish(str_msg)
        #print('updating the position')
        if idx%100==1:
            rospy.loginfo("GPS data "+ str(idx) + " : " + str(dict_GPS))
        # rate.sleep()

    except Exception as e:
        print(' Error in reading the GPS sensor!!')
        print('Exception: ',e)
    except KeyboardInterrupt:
        break






