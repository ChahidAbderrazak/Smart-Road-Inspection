import os
import numpy as np

def load_json(filename):
	try:
		if os.path.exists(filename):
			import json
			f = open(filename,)
			data = json.load(f)
			f.close()
		else:
			data=[]
		return data
	except:
		msg = f'\n\n Error: The JSON file <{filename}> cannot be read correctly!!  \
			       \n --> a new Jason file will be created!!    '
		print(msg)
		# raise ValueError(msg)
		return []


def save_json(json_string, filename):
	import json
	try:
		# Using a JSON string
		with open(filename, 'w') as outfile:
			json.dump(json_string, outfile,indent=2)
			return 0
	except:
		return 1

def get_sensor_filename(sensor_name, frame, config):
	try:
		scene_setup = '__' + config['Scenario']+ '-' + config['USE_CASE']
	except:
		scene_setup = '__Unknown'
	time_tag =  str(get_time_tag(type=1))
	return time_tag + scene_setup + '__' + sensor_name + '__' + str(frame)

def get_time_tag(type=1):
	from datetime import datetime
	today = datetime.now()
	if type==0:
			return today.strftime("__%Y-%m-%d")
	else:
			return today.strftime("%Y-%m-%d-%Hh-%Mmin-%Ssec")

def create_new_folder(DIR):
	if not os.path.exists(DIR):
		os.makedirs(DIR)

def update_table(old_table, entry, filename):
	e=entry['token']
	for ID in old_table:
		if ID['token']==entry['token']:
			msg = f'\n\n Error: The token <{e}> already exist in the table'
			print(msg)
			# raise ValueError(msg)
			return 1
	table = old_table + [entry]
	save_json(table, filename)

def get_sample_token():
	return 0

#################	 LOG TABLE	################# 
def get_log_token(config):
	return config['vehicle'] +'__'+  config['location'] +'__'+ config['Scenario']

def build_log_table_entry(token,vehicle,location):
		from datetime import datetime
		today = datetime.now()
		logfile = vehicle + '_' + today.strftime("%Y-%m-%d-%Hh-%Mmin")
		date_captured=today.strftime("%Y-%m-%d-%Hh-%Mmin-%Ssec")
		entry = {
		"token": token,
		"logfile": logfile,
		"vehicle": vehicle,
		"date_captured": date_captured,
		"location": location
		}
		return entry

#################	 SCENE TABLE	 ################# 
def get_scene_token(config=''):
	try:
		scene_setup = '__' + config['Scenario']+ '-' + config['USE_CASE']
	except:
		scene_setup = ''
	
	time_tag = get_time_tag(type=1)
	return time_tag + scene_setup

def build_scene_table_entry(name,token,log_token,scene_table, first_sample_token, last_sample_token, config):
	nbr_samples = len(scene_table)
	entry = {
	"token": token,
	"log_token": log_token,
	"nbr_samples": nbr_samples,
	"first_sample_token": first_sample_token,
	"last_sample_token": last_sample_token,
	"name": name,
	"description": config['description']
	}
	return entry

#################	 SAMPLE TABLE	 ################# 
def get_sample_token(scene_, config=''):
	try:
		scene_setup = '__' + config['Scenario']+ '-' + config['USE_CASE']
	except:
		scene_setup = '__Unknown'
	time_tag = str(scene_['frame'])
	return time_tag + scene_setup

def build_sample_table_entry(token,timestamp,prev_sample, next_sample, scene_token):
	entry = {
	"token": token,
	"timestamp": timestamp,
	"prev": prev_sample,
	"next": next_sample,
	"scene_token": scene_token
	}
	return entry

#################	 SAMPLE TABLE	 ################# 
def get_sample_data_token(sample_token, list_sensors):
	if sample_token=='':
		return ''
	else:
		frame_tag = 'sdata__' + sample_token
		sensor_tag = '__sensors' + str(np.sum([list_sensors[v]>0 for v in list_sensors]))
		return frame_tag + sensor_tag

def build_sample_data_table_entry(token,timestamp, sample_token, ego_pose_token, calibrated_sensor_token, \
	                               fileformat, is_key_frame, prev_sample, next_sample, filename, height=0, width=0):
	entry = {
	"token": token,
	"sample_token": sample_token,
	"ego_pose_token": ego_pose_token,
	"calibrated_sensor_token": calibrated_sensor_token,
	"timestamp": timestamp,
	"fileformat": fileformat,
	"is_key_frame": is_key_frame,
	"height": height,
	"width": width,
	"filename": filename,
	"prev": prev_sample,
	"next": next_sample,
	}
	return entry

#################	 EGO-POS TABLE	 ################# 

def get_ego_pose_token(sample_token):
	return 'pos__' + sample_token

def build_ego_pose_table_entry(token, timestamp, rotation=[], translation=[]):
	entry ={
	"token": token,
	"timestamp": timestamp,
	"rotation": rotation ,
	"translation": translation
	}
	return entry

#################	 CLIBRATIO TABLE	 ################# 

def get_calib_token(sample_token):
	return 'calib__' + sample_token

def build_calib_table_entry(token, sensor_token,  translation=[], rotation=[], camera_intrinsic=[]):
	entry ={
	"token": token,
	"sensor_token": sensor_token,
	"translation": translation,
	"rotation": rotation,
	"camera_intrinsic":camera_intrinsic
	}
	return entry


#################	 SENSOR TABLE	 ################# 
def get_sensor_token(sensor_name, config):
	return config['vehicle'] + '__' + sensor_name

def get_sensor_modality(sensor_name):
	import re
	if re.search('CAM', sensor_name, re.IGNORECASE):
		modality ="camera"
		fileformat="jpg"

	elif re.search('LIDAR', sensor_name, re.IGNORECASE):  
		modality ="lidar"
		fileformat="bin"

	elif re.search('RADAR', sensor_name, re.IGNORECASE):
		modality ="radar"
		fileformat="pcd"

	elif re.search('GNSS', sensor_name, re.IGNORECASE):
		modality ="other"
		fileformat="pkl"
	else:
		print(f'\n Error: The sensor name <{sensor_name}> is not defined!!!')
		modality ="Undefined"
		fileformat=""
	
	return  modality, fileformat

def build_sensor_table_entry(token, channel, modality):
	entry ={
			"token": token,
			"channel": channel,
			"modality": modality
			}
	return entry

#################	 INSTANCE TABLE	 ################# 

def get_instance_token(scene_token):
	return 'ann-scene__' + scene_token


def build_instance_table_entry(token, category_token, nbr_annotations, first_annotation_token, last_annotation_token):
	entry ={
		"token": token,
		"category_token": category_token,
		"nbr_annotations": nbr_annotations,
		"first_annotation_token": first_annotation_token,
		"last_annotation_token": last_annotation_token
		}
	return entry

#################	 ANNOTATION TABLE	 ################# 
def get_sample_annotation_token(sample_data_token):
	if sample_data_token == '':
		return ''
	else:
		return 'ann-sdata__' + sample_data_token

def build_sample_annotation_table_entry(token, sample_token, instance_token, visibility_token, attribute_tokens,\
													translation, size, rotation, prev_sample_annotation, next_sample_annotation,num_lidar_pts,\
													num_radar_pts):
	entry ={
		"token": token,
		"sample_token": sample_token,
		"instance_token": instance_token,
		"visibility_token": visibility_token,
		"attribute_tokens": attribute_tokens,
		"translation": translation,
		"size": size,
		"rotation": rotation,
		"prev": prev_sample_annotation,
		"next": next_sample_annotation,
		"num_lidar_pts": num_lidar_pts,
		"num_radar_pts": num_radar_pts
		}
	return entry


