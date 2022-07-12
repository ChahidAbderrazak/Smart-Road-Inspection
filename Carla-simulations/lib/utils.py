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

def get_time_tag(type=0):
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
	for ID in old_table:
		if ID['token']==entry['token']:
			e=entry['token']
			msg = f'\n\n Error: The token <{e}> already exist in the table'
			print(msg)
			# raise ValueError(msg)
	table = [entry] + old_table
	save_json(table, filename)

def get_sample_token():
	return '0'

#################	 LOG TABLE	################# 
def get_log_token(log_table):
	return str(len(log_table))

def build_log_table_entry(token,vehicle,location):
		from datetime import datetime
		today = datetime.now()
		logfile = vehicle + '_' + today.strftime("%Y-%m-%d-%Hh-%Mmin")
		date_captured=today.strftime("%Y-%m-%d-%Hh-%Mmin-%ssec")
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
		scene_setup = '__' + config['Scenario']+ '-' + config['Used_Case']
	except:
		scene_setup = ''
	
	time_tag = get_time_tag(type=1)
	return time_tag + scene_setup

def build_scene_table_entry(name,token,log_token,scene_table, config):
	nbr_samples = len(scene_table)
	first_sample_token = get_sample_token(scene_table[0], config=config)
	last_sample_token = get_sample_token(scene_table[-1], config=config)
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
		scene_setup = '__' + config['Scenario']+ '-' + config['Used_Case']
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
def get_sample_data_token(scene_, config=''):
	try:
		scene_setup = '__' + config['Scenario']+ '-' + config['Used_Case']
	except:
		scene_setup = '__Unknown'
	time_tag = str(scene_['frame'])
	return time_tag + scene_setup

def build_sample_data_table_entry(token,timestamp,prev_sample, next_sample, scene_token):
	entry = {
	"token": token,
	"timestamp": timestamp,
	"prev": prev_sample,
	"next": next_sample,
	"scene_token": scene_token
	}
	return entry


#################	 SENSOR TABLE	 ################# 
	['Depth_Camera' 'GNSS_Sensor' 'IMU_Sensor' 'Lidar_Sensor'
 'Obstacle_Sensor' 'RGB_Camera_Back' 'RGB_Camera_Front' 'Radar_Sensor'
 'Semantic_Lidar_Sensor']
