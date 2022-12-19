#!/usr/bin/env python
import os, sys
import threading
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import cv2
from cv_bridge import CvBridge
import time
import numpy as np

# from lib.config_parameters import *
# from lib.utils import *


sensor_frame=0
scene_count=0


def get_timestamp():
    # from datetime import datetime
    # # Getting the current date and time
    # dt = datetime.now()
    # # getting the timestamp
    # ts = datetime.timestamp(dt)
    import time
    ts=time.time()
    return ts

def get_time_tag(type=1):
    from datetime import datetime
    today = datetime.now()
    if type == 0:
        return today.strftime("__%Y-%m-%d")
    else:
        return today.strftime("%Y-%m-%d-%Hh-%Mmin-%Ssec")

# create json file
def get_file_names(config, fr=''):
    if fr == '':
        time_tag = str(get_time_tag(type=1))
        filename_strg =  time_tag + '_.json'
    else:
        time_tag = str(get_time_tag(type=1))
        filename_strg = 'Fr' + fr + '_' + time_tag + '_.json'
    dir_storage = ""
    return filename_strg, dir_storage


###### intialize paramaters
configuration = {"vehicle": "node2", "location": "Oshawa",
					       "description": "HAIS- road Inspection Node 2"}
                           # Input parameters 
fs=1 #5   # sampling frequancy in Hz 
data_root= "/home/hais/catkin_ws/src/upload" #"/home/hais/Desktop/data-collection/upload"#  #absolute path to log dir
node_name= "hais_data_logger_node"
stop_threads = False
dict_frame = {}  # dictionary of sensor data corresponding to a single frame
dict_fr_list = [{}]  # list of individual dictionaries of sensors" data for all frames
# PORT_NAME = '/dev/ttyUSB2'

# create json file
filename_strg, _ = get_file_names(configuration)
mission_filename = os.path.join(data_root, "missions", filename_strg)
sensor_frame=0
car_location=''

# devces intilization
bridge = CvBridge()


def load_json(filename):
    if os.path.exists(filename):
        import json
        f = open(filename)
        data = json.load(f)
        f.close()
        # if len(data)%500==1:
        print("- Previous data has " + str(len(data)) + " sensors recodrs")
    else:
        data = []
    
    return data
    
def save_json(json_string, filename):
    """
    json_string=[{}, {}]
    filename="data/file.json"
    """
    import json
    try:
        # Using a JSON string
        with open(filename, "w") as outfile:
            json.dump(json_string, outfile, indent=2)
            return 0
    except:
        return 1

def create_databse_folders(data_root):
    global json_path
    json_path = os.path.join(data_root,"missions")
    lidar_data_path = os.path.join(data_root,"sweeps","LIDAR")
    image_data_path = os.path.join(data_root,"sweeps","3D_CAMERA")
    create_new_folder(json_path)
    create_new_folder(lidar_data_path)
    create_new_folder(image_data_path)

def color_callback(color_msg_data):
    global color_img
    # get GPS 3D Camera RGB image
    color_img = bridge.imgmsg_to_cv2(color_msg_data, desired_encoding='passthrough') 


def save_RGB_camera_data(color_img):
    global car_location, sensor_frame, dict_fr_list, scene_count
    # save 3D Camera RGB image
    sensor_name="3D_CAMERA"
    ext= '.jpg' #'.npy' #
    filename_strg = str(get_sensor_filename(sensor_name, sensor_frame, tag='rgb')) + ext
    filename = os.path.join(data_root, "sweeps", sensor_name, filename_strg)

    # np.save(filename, color_img)
    cv2.imwrite(filename, color_img)
    # update outputs
    dict_frame = add_data_to_pipeline(sensor_frame)
    sensor_frame = sensor_frame + 1
    dict_frame["description"] = configuration["description"]
    dict_frame["timestamp"] = get_timestamp()
    dict_frame["scene"] = scene_count
    dict_frame["sensor_name"] = "CSI_CAMERA"
    dict_frame["position"] = {"Translation": car_location,
                              "Rotation": []}
    dict_frame["calibration"] = {"Translation": [], "Rotation": [], "Camera_intrinsic": ""}
    dict_frame["fileformat"] = ext
    dict_frame["filename"] = str(filename)
    dict_frame["meta_data"] = ""

    # update the mission file

def depth_callback(depth_msg_data):
    global depth_img
    # get/save 3D Camera Depth image
    depth_img = bridge.imgmsg_to_cv2(depth_msg_data, desired_encoding='passthrough')
   
def save_depth_camera_data(depth_img):
    global car_location, sensor_frame, dict_fr_list, scene_count
    # get/save 3D Camera Depth image
    sensor_name="3D_CAMERA"
    ext= '.jpg' # '.npy' #
    filename_strg = str(get_sensor_filename(sensor_name, sensor_frame, tag='depth')) + ext
    filename = os.path.join(data_root, "sweeps", sensor_name, filename_strg)

    # np.save(filename, depth_img)
    cv2.imwrite(filename, depth_img)
    # update outputs
    dict_frame = add_data_to_pipeline(sensor_frame)
    sensor_frame = sensor_frame + 1
    dict_frame["description"] = configuration["description"]
    dict_frame["timestamp"] = get_timestamp()
    dict_frame["scene"] = scene_count
    dict_frame["sensor_name"] = "CSI_CAMERA"
    dict_frame["position"] = {"Translation": car_location,
                              "Rotation": []}
    dict_frame["calibration"] = {"Translation": [], "Rotation": [], "Camera_intrinsic": ""}
    dict_frame["fileformat"] = ext
    dict_frame["filename"] = str(filename)
    dict_frame["meta_data"] = ""

def scan_callback(scan_msg_data):
    global scan_ranges
    scan_ranges= np.asarray(scan_msg_data.ranges) #save scan as np array

def save_lidar_data(scan_ranges):
    global car_location, sensor_frame, dict_fr_list
    # save lidar data
    sensor_name="LIDAR"
    ext='.npy'
    filename_strg = str(get_sensor_filename(sensor_name, sensor_frame, tag='depth')) + ext
    filename = os.path.join(data_root, "sweeps", sensor_name, filename_strg)

    np.save(filename, scan_ranges)
    print('\n\n\n  lidar saved in ', filename)
    rospy.loginfo("Data Logged @"+str(time.ctime()))

    # update outputs
    dict_frame = add_data_to_pipeline(sensor_frame)
    sensor_frame = sensor_frame + 1
    dict_frame["description"] = configuration["description"]
    dict_frame["timestamp"] = get_timestamp()
    dict_frame["scene"] = scene_count
    dict_frame["sensor_name"] = "CSI_CAMERA"
    dict_frame["position"] = {"Translation": car_location,
                              "Rotation": []}
    dict_frame["calibration"] = {"Translation": [], "Rotation": [], "Camera_intrinsic": ""}
    dict_frame["fileformat"] = ext
    dict_frame["filename"] = str(filename)
    dict_frame["meta_data"] = ""

    dict_fr_list.append(dict_frame)

def gps_callback(gps_data):
    global car_location
    car_location= gps_data
    # print(f"\n - car_location ={car_location}")
    # # update outputs
    # dict_frame = add_data_to_pipeline(sensor_frame)
    # sensor_frame = sensor_frame + 1
    # dict_frame["description"] = configuration["description"]
    # dict_frame["timestamp"] = get_timestamp()
    # dict_frame["scene"] = scene_count
    # dict_frame["sensor_name"] = "GPS_SENSOR"
    # dict_frame["position"] = {"Translation": car_location,
    #                           "Rotation": []}
    # dict_frame["calibration"] = {"Translation": [], "Rotation": [], "Camera_intrinsic": ""}
    # dict_frame["fileformat"] = ""
    # dict_frame["filename"] = ""
    # dict_frame["meta_data"] = "

def get_sensor_filename(sensor_name, frame, tag=''):
    time_tag = time.ctime().replace(" ", "_") 
    return time_tag  + "__" + tag + "__" + sensor_name + "__" + str(frame) 

def create_new_folder( DIR ):
    import os 
    if not os.path.exists( DIR ):
        os.makedirs( DIR )

def add_data_to_pipeline(frame):
    global car_location, sensor_frame, dict_fr_list, scene_count
    # function to add data to main sensor data dictionary based on corresponding frame
    global dict_fr_list, dict_frame
    if dict_frame == {}:
        dict_frame["frame"] = frame
        scene_count+=1

    elif frame != dict_frame["frame"]:
        dict_fr_list.append(dict_frame)  # main list with all frames
        #print(dict_fr_list)
        dict_frame = {}  # individual frame
        dict_frame["frame"] = frame
    return dict_frame

def save_mission_json_file():
    global dict_fr_list, mission_filename
    if len(dict_fr_list)>5:    
        old_dict = load_json(mission_filename)
        combined_dict = old_dict + dict_fr_list
        save_json(combined_dict, mission_filename)
        dict_fr_list = []
        if len(combined_dict)%100==0:
            # print(f"\n - Saving {str(len(combined_dict))}  frames in {mission_filename}" )
            print("\n - Saving " + str(len(combined_dict))+" frames in "+str(mission_filename) )
        
def init_database():
    global mission_filename, car_location, sensor_frame, disp
    disp=False
    dict_fr_list = []
    # get the car position
    # car_location= get_gps_data()
    car_location= [-1,-1,-1]
    
    # define the log file
    data_log = load_json(mission_filename)
    sensor_frame = 1+len(data_log)
    if sensor_frame==0:
        save_json([{}], filename)
    # set up folder to save data locally
    create_databse_folders(data_root)

    # create the mission info
    info_file_path=os.path.join(data_root, 'info.json')
    save_json(configuration, info_file_path)

    # display
    print("configuration=", configuration)
    print("filename=", mission_filename)
    print("sensor_frame=", sensor_frame)



#define the sensors subscriber
###-----------------------  3D CAMERA  -----------------------###
color_sub= rospy.Subscriber("/camera/color/image_raw", Image, color_callback)
depth_sub= rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_callback)

###-------------------------  LIDAR   ------------------------###
scan_sub= rospy.Subscriber("/scan", LaserScan, scan_callback)

###-------------------------  LIDAR   ------------------------###
gps_sub= rospy.Subscriber("/gps_location", String, gps_callback)


# intialization 
rospy.init_node(node_name) 
rate_slow = rospy.Rate(fs)
init_database()

#Start listening for subscriptions without halting the program flow
t1= threading.Thread(target=rospy.spin) #thread for rospy.spin
t1.start() 
rospy.loginfo('HAIS Node Initialized '+str(fs)+'Hz')
while not rospy.is_shutdown():
    try:
        # get GPS 3D Camera RGB image
        print('car location=', car_location)

        # save LIDAR
        save_lidar_data(scan_ranges)

        # save RGB camera
        save_RGB_camera_data(color_img)

        # save RGB camera
        save_depth_camera_data(depth_img)

        # save mission file
        save_mission_json_file()

    except Exception as e:
        print('Exception: ',e)
    except KeyboardInterrupt:
        break

## terminate thread
t1.join()

