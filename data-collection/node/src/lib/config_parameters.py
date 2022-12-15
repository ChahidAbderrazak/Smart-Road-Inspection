import sys, os
import cv2
import pyrebase
from lib.utils import get_file_names, create_new_folder

###### Sensors
cam0 = cv2.VideoCapture(0)														# Main Camera for the road
cam_right= cv2.VideoCapture(1)												# RIGHT Camera for the right lane marker
cam_left= cv2.VideoCapture(2)													# LEFT Camera for the left lane marker
PORT_NAME='/dev/ttyUSB0'															# Lidar ports
scan_type='normal' #'express'# 												# lidar scanning mode

###### intialize paramaters
configuration = {	"vehicle": "car1", 
									"node": "Node1", 
									"location": "Oshawa", 
									"description": "HAIS- datalogger for road Inspection"}
data_root = "../upload"  															# storage path on local device


# intialize global variables
dict_frame = {}  																			# dictionary of sensor data corresponding to a single frame
dict_fr_list = [{}]																		# list of individual dictionaries of sensors" data for all frames
sensor_frame=0																				# intial frame ID
scene_count=0																					# intial scene ID

# create tmp folder 
root=os.path.join(data_root, 'tmp')
create_new_folder(root)

# create json file
data_root=os.path.join(data_root, configuration['node'])
filename_strg, _ = get_file_names(configuration)
filename = os.path.join(data_root, "missions", filename_strg)
