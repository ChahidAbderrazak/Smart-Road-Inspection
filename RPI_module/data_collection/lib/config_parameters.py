import sys, os
from lib.utils import get_file_names
###### intialize paramaters
configuration = {"vehicle": "car1", "location": "Oshawa",
					       "description": "HAIS- road Inspection Node 1"}
#data_root = "HAIS_DATABASE"  # storage path on local device
data_root = "/media/hais/355A-55FB/HAIS_DATABASE" # storage path on HHD device
stop_threads = False
dict_frame = {}  # dictionary of sensor data corresponding to a single frame
dict_fr_list = [{}]  # list of individual dictionaries of sensors" data for all frames
#img_size=(1000, 1000)
PORT_NAME = '/dev/ttyUSB0'

# create json file
filename_strg, _ = get_file_names(configuration)
filename = os.path.join(data_root, "missions", filename_strg)
sensor_frame=0
