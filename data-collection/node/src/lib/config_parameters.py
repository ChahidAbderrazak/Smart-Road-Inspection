import sys, os
import cv2
###### Sensor
PORT_NAME='/dev/ttyUSB0'													# Lidar ports
scan_type='normal' #'express'# 										# lidar scanning mode

###### intialize paramaters
configuration= {"vehicle": "car1", 
								"node": "Node1", 
								"location": "Oshawa", 
								"description": "HAIS- datalogger for road Inspection"}

fs=5 #1  																				# sampling frequancy in Hz 
data_root = "/home/pi/Desktop/upload"    				# RPi: storage path on local device
# data_root = "/home/hais/Desktop/upload"     # Jetson; storage path on local device

# intialize global variables
dict_frame = {}  																# dictionary of sensor data corresponding to a single frame
dict_fr_list = [{}]															# list of individual dictionaries of sensors" data for all frames
sensor_frame=0																	# intial frame ID
scene_count=0																		# intial scene ID
car_location=[-1,-1,-1]										                # intial car position (waiting for the GPS to start)
data_root=os.path.join(data_root, configuration['node'])
