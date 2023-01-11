#!/usr/bin/env python
import os
import sys
import time
import cv2

###### ROS packages
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

cam_right_pub= rospy.Publisher("/cam_right", Image, queue_size=1)
cam_left_pub= rospy.Publisher("/cam_left", Image, queue_size=1)
rospy.init_node("Side_Cameras")

# devces intilization
bridge = CvBridge()
cam_right= cv2.VideoCapture(1)												# RIGHT Camera for the right lane marker
cam_left= cv2.VideoCapture(5)												# LEFT Camera for the left lane marker
# rate  = rospy.Rate(60) #60 Hz
idx=0
while not rospy.is_shutdown():
    idx+=1
    # right-sided camera
    try:
        ret, rgb_frame=cam_right.read()
        msg = bridge.cv2_to_imgmsg(rgb_frame, encoding="bgr8")
        cam_right_pub.publish(msg)
        # display
        if idx%100==1:
            rospy.loginfo("Right lane marker data "+ str(idx) + " : image shape " + str(rgb_frame.shape))
        # rate.sleep()

    except Exception as e:
        print(' Error in reading the Right Camera sensor!!')
        print('Exception: ',e)
    except KeyboardInterrupt:
        break

    # left-sided camera
    try:
        ret, rgb_frame=cam_left.read()
        msg = bridge.cv2_to_imgmsg(rgb_frame, encoding="bgr8")
        cam_left_pub.publish(msg)
        # display
        if idx%100==1:
            rospy.loginfo("Left lane marker data "+ str(idx) + " : image shape " + str(rgb_frame.shape))
        # rate.sleep()

    except Exception as e:
        print(' Error in reading the Right Camera sensor!!')
        print('Exception: ',e)
    except KeyboardInterrupt:
        break

    






