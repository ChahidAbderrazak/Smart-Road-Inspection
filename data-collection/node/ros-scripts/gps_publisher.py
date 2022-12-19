## # !/usr/bin/env python
import rospy
from std_msgs.msg import String
# from NANO_module.data_collection.lib_RPI.Gps import *

str_msg= String()

str_pub= rospy.Publisher("/gps_location", String, queue_size=10)
rospy.init_node("GPS_node")

# rate  = rospy.Rate(1) #1 Hz
# rate  = rospy.Rate(5) #5 Hz
idx=0
while not rospy.is_shutdown():
    idx+=1
    try:
        # car_location = get_gps_data()
        car_location=[12, 34, -1]
        # print('\n - car_location =', car_location)
        str_msg.data= str(car_location)
        str_pub.publish(str_msg)
        rospy.loginfo("GPS position "+ str(idx) + " : " + str(car_location))
        # rate.sleep()

    except Exception as e:
        print('Exception: ',e)
    except KeyboardInterrupt:
        break






