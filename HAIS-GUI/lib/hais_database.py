# HAIS-Bot.self.nusc=
# Code written by Abderrazak Chahid, 2022.
# NB: some part/structure are inspired/used from nuScenes
# code source: https://github.com/nutonomy/nuscenes-devkit/blob/master/python-sdk/nuscenes/nuscenes.py

import os, sys
import time
import json
from tqdm import tqdm
import pickle
import os.path as osp
import numpy as np
from glob import glob
from lib import utils

def set_timestamp():
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

def get_sensor_filename(sensor_name, frame,time_tag='', tag=''):
		if time_tag=='':
				time_tag = str(set_timestamp())+"_tmstmp_"+ get_time_tag(type=1)
		return time_tag	+ "__" + sensor_name + "_"+ tag +"_" + str(frame)

def get_timestamp(path):
	stamp_str=path.split('_tmstmp_')
	return float(stamp_str[0])

# def get_timestamp(path):
# 	stamp_str=path.split('sec_')
# 	return stamp_str[0]

def get_new_sensor_dict(sensor_name, filename	,sensor_frame, sensor_scene, 
										car_location, timestamp, description, meta_data=""):
		_, ext = os.path.splitext(filename)
		dict_frame={}
		dict_frame["frame"]=sensor_frame
		dict_frame["description"]=description
		dict_frame["timestamp"]=timestamp
		dict_frame["scene"]=sensor_scene
		dict_frame["sensor_name"]=sensor_name
		dict_frame["position"]={"Translation": car_location, 
														"Rotation": []}
		dict_frame["calibration"]={"Translation": [], "Rotation": [], "Camera_intrinsic": ""}
		dict_frame["fileformat"]=ext
		dict_frame["filename"]=filename
		dict_frame["meta_data"]=meta_data
		
		return dict_frame

def timestamp_from_filename(sensor_name, curr_sample):
	global data_dict
	idx=curr_sample[sensor_name][1]
	filename_path=data_dict[sensor_name][idx][2]
	timestamp=get_timestamp(os.path.basename(filename_path) )
	return timestamp

def update_curr_dict(curr_sample, sensor_name):
	if curr_sample[sensor_name][1]<curr_sample[sensor_name][2]:
				curr_sample[sensor_name][1]+=1
				
	# 			return False
	# else:
	# 	return True
	
def fill_sample_new_data(sensor_name, curr_sample, frame, scene, disp=False):
	global 	data_dict, scene_sensor, car_location, configuration, \
					timestamp_line, time_idx
	
	sensor_type=curr_sample[sensor_name][0]
	if disp:
		print(f'\n checking [frame={frame},  sensor={sensor_name}, sensor_type={sensor_type}')

	if sensor_type=='files': # sensors with data files [Camera, Lidar, etc]
		if disp:
			print(f'\t -> processing sensors with data files [{sensor_name}]')
		
		# get sensor info
		idx=curr_sample[sensor_name][1]
		try:
			filename_path=data_dict[sensor_name][idx][2]
		except Exception as e:
			if disp:
				print(f'\n Error in updating {sensor_name}!! \n Exception: {e}')
			return {}
		
		timestamp=get_timestamp(os.path.basename(filename_path) )
		if timestamp>timestamp_line[time_idx]:# data IS NOT within the frame timestamp 
			if disp:
				print(f'\n --> sensor_name={sensor_name} data does not belong to  frame   {frame}!')
			return {}
		
		else: # data IS within the frame timestamp 
			print(f'\n  \t [{int(100*time_idx/len(timestamp_line))} %] {timestamp} | {timestamp_line[time_idx]} --> sensor_name={sensor_name} ')
			if disp:
				print(f'\n --> sensor_name={sensor_name} \n {filename}')
				print(f'\n - saving data_ sample ...')

			
			# get the frame info
			description=""
			filename=os.path.join('sweeps', filename_path.split('sweeps')[1][1:])
			# input(f'\n flag: filename={filename}')
			
			# check the 3D camera RGB/Gray 
			if "3D" in sensor_name:
				filename_path=glob(filename_path[:-4] + '_*')[0]
				meta_data=os.path.join('sweeps', filename_path.split('sweeps')[1][1:])
			else:
				meta_data=""

			# push new data pipeline
			update_curr_dict(curr_sample, sensor_name)

			# build the sample data 
			sensor_dict = get_new_sensor_dict(sensor_name, 
																				filename=filename,
																				sensor_frame=frame, 
																				sensor_scene=scene, 
																				car_location=car_location, 
																				timestamp=timestamp, 
																				description=description, 
																				meta_data=meta_data)
			if disp:
				print(f'\t\t -> new sensor frame= [{sensor_dict}]')

			return  sensor_dict
	
	elif sensor_type=='json':# sensors with Json recordings [IMU/GPS]
		if disp:
			print(f'\t -> processing sensors with Json recordings [{sensor_name}]')
		
		# get sensor info
		sensor_dict=curr_sample[sensor_name][5]
		timestamp=sensor_dict["timestamp"]
		if timestamp>timestamp_line[time_idx]:# data IS NOT within the frame timestamp 
			if disp:
				print(f'\n --> sensor_name={sensor_name} data does not belong to  frame   {frame}!')
			return {}

		else:
			print(f'\n  \t [{int(100*time_idx/len(timestamp_line))} %] {timestamp} | {timestamp_line[time_idx]} --> sensor_name={sensor_name} ')
			# update the sensor frame details
			car_location=sensor_dict["position"]["Translation"]
			# update the mission dict
			sensor_dict["frame"]=frame
			sensor_dict["scene"]=scene

			# push new data pipeline
			update_curr_dict(curr_sample, sensor_name)

			return sensor_dict

class HAIS_database:
	"""
	Database class for HAIS-bot to store compatible data structue with the nuScenes database structure.
	"""
	def __init__(self,dataroot: str = 'data/',
										version: str = 'data/',
										verbose: bool = False):
			"""
			Loads/Collected data and saved the json tables and filder/files strucutre 
			:param config: Collected data setup configuration: inspection node, desctiption,etc.
			:param version: Version to load (e.g. "v1.0", ...).
			:param dataroot: Path to the raw data dictionnary.
			:param verbose: Whether to print status messages during load.
			:param map_resolution: Resolution of maps (meters).
			"""
			self.dataroot=dataroot
			self.version = version
			self.verbose = verbose
			self.nusc=None
			self.config_file=os.path.join(self.dataroot, 'info.json')
			self.config=self.load_config()
			self.node_name=self.config["vehicle"]

			# load/create database
			self.err, self.msg = self.load_create_database()

			# display
			self.display()

	@property
	def table_root(self) -> str:
			""" Returns the folder where the tables are stored for the relevant version. """
			return osp.join(self.dataroot, self.version)

	def __load_table__(self, table_name) -> dict:
			""" Loads a table. """
			filename=osp.join(self.table_root, '{}.json'.format(table_name)) 
			if not osp.exists(filename):
				# create an empty json file
				self.create_new_folder(osp.dirname(filename))
				with open(filename, 'w') as outfile:
					json.dump([], outfile,indent=2)
			# open JSON table
			with open(filename) as f:
					table = json.load(f)
			return table

	def load_config(self):
		try:
			config=self.load_json(self.config_file)
			return config

		except Exception as e:
			print(f'\n\n Exception: {e}')
			sys.exit(0)

	def display(self):
		print(f'\n########################################################')
		print(f'##            HAIS database              ')
		print(f"## - Node= {self.node_name} 	")
		print(f"## - dataset= {self.version} 	\n## - root= {self.dataroot}")
		print(  f'########################################################')
	
	def convert_drone_database(self):
			# If Drone, build the predefined data structure od HAIS node
			if 'drone' in self.node_name:
				print( f'\t - This data is collected by a drone [{self.node_name}] and needs to be converted')
				try:
					from lib import dji_drone
				except:
					import dji_drone
				dji_drone.build_Hais_data_structure(	self.dataroot)
			# building the json tables
			self.create_database_table()

	def convert_JetsonNano_database(self, root, disp=False):
		global 	time_idx, data_dict, scene_sensor, scene_sensor, frame, scene, \
						car_location, timestamp_line, time_idx, configuration
		print(f'\n ---------Converting the log file to mission json file ----------- ')
		# initialization
		scene_sensor=[]
		scene=0
		frame=0
		time_idx=1
		car_location=[-1, -1,-1]
		timestamp_line=[]
		# search the sensor files/data
		tracker_json=[ path for path in glob(os.path.join(root, 'log', '*.json')) if os.path.isfile(path)]
		list_sensors=[ os.path.basename(path) for path in glob(os.path.join(root, 'sweeps', '*')) if os.path.isdir(path)]
		print(f'\n list_sensors= {list_sensors}')
		data_dict={}
		curr_sample={}
		mission_data_list=[]
		N=0 # maximal number of files
		for sensor_name in list_sensors:
			list_files=[ (sensor_name, get_timestamp(os.path.basename(path)), path) \
										for path in glob(os.path.join(root, 'sweeps', sensor_name, '*')) \
											if os.path.isfile(path) and not '_depth' in path]
			list_files.sort()

			# print(f'\n - {sensor_name} has {len(list_files)} file')
			data_dict.update({sensor_name:list_files})
			if len(list_files)>0:
				t0=get_timestamp(os.path.basename(list_files[0][2]))
				tN=get_timestamp(os.path.basename(list_files[-1][2]))
				curr_sample.update({sensor_name:['files', 0,len(list_files),t0, tN]})
				# get sensors timeline
				timeline_=[frame_[1] for frame_ in list_files]
				# update the timestamp_line
				timestamp_line+=timeline_

		# compile/build mission Json file based on all files in the root
		timestamp_line.append(set_timestamp())
		time_idx=0
		for mission_json_path in tracker_json:
			mission_dict=utils.load_json(mission_json_path)

			# get the json sensors list
			for data_sample in mission_dict:
				sensor_name=data_sample["sensor_name"]
				sensor_frame=data_sample["frame"]
				
				# load list of frames of the existing sensors
				try: 
					list_files=data_dict[sensor_name]
				except: # first data of <sensor_name>
					list_files=[]
					t0=data_sample["timestamp"]

				# update frame
				tN=data_sample["timestamp"]
				timestamp_line.append(tN)
				list_files.append(sensor_frame)
				# print(f'\n - {sensor_name} has {len(list_files)} file')
				data_dict.update({sensor_name:list_files})
				curr_sample.update({sensor_name:['json', 0,len(list_files),t0, tN, data_sample]})

			# update the mission json file
			car_location=[-1,-1,-1]
			timestamp_line=np.unique(timestamp_line)
			timestamp_line.sort()
			
			# save the list of sensor
			list_sensors=list(curr_sample.keys())
			sensors_file=os.path.join(root, 'sensor.json')
			utils.save_json(list_sensors, sensors_file)
			# define the timeline sensors with maximal number of timestamps
			for sensor_name in curr_sample.keys():
				Ns=len(data_dict[sensor_name])
				print(f'\n - sensor_name has {Ns} data samples')
				if Ns>N: #"CAMERA" in sensor_name and 
					N=Ns
					ref_sensor_name=sensor_name

			# display sensors file summary 
			print(f'\n Generate the mission json file. \
							\n - list of  sensor= {curr_sample.keys()} \
							\n - largest ref sensor data = {ref_sensor_name} with {N} files \
							\n - timestamp_line = {len(timestamp_line)} time sample \
							\n - GPS/IMU json have {len(mission_dict)} frames ')
			


			# loop over al the available sensor data
			stop_flag=False
			new_frames_list_old=[]
			while(not stop_flag):
				# print(f'\n curr_sample={curr_sample}')
				
				new_frames_list=[]
				for sensor_name in curr_sample.keys(): # sensor with data files
					# get new sensors sample_data
					new_data= fill_sample_new_data(sensor_name=sensor_name, 
																				curr_sample=curr_sample,
																				frame=frame, 
																				scene=scene,
																				disp=disp)
					if new_data!={}:
						mission_data_list+=new_frames_list
						frame+=1
						if ref_sensor_name==sensor_name:
							scene+=1
						new_frames_list.append(new_data)
					else:
						if disp:
							print(f'\n {sensor_name} has empty data at {timestamp_line[time_idx]}!!')
				# update the mission dict
				# print(f'\n new_frames_list={new_frames_list}')
				if new_frames_list_old!=new_frames_list:
					mission_data_list+=new_frames_list
					new_frames_list_old=new_frames_list
				else:
					time_idx+=1
					# check the end of the timeline
					if time_idx==len(timestamp_line):
						stop_flag=True
					# print(f'\n timestamp time is empty of data. Step forward={timestamp_line[time_idx]}')
					# print(f'\n skipped sensors data of [frame={frame},  sensor={sensor_name}]')

				# # [Dev] stop the loop
				# if scene>50:
				# 		stop_flag=True

			# display 
			print(f'\n ==> mission file create [ {len(mission_data_list)} data samples \n  head={mission_data_list[:10]}')
			mission_json=mission_json_path.replace('log', 'missions')
			utils.create_new_directory(os.path.dirname(mission_json))
			utils.save_json(mission_data_list, mission_json)

	def create_database_table(self):
		# building the json tables
		self.table_names = ['category', 'attribute', 'visibility', 'instance', 'sensor', 'calibrated_sensor',
												'ego_pose', 'log', 'scene', 'sample', 'sample_data', 'sample_annotation', 'map', 'labels_config']
		self.scenes_path = os.path.join(self.dataroot, "missions")
		self.log_path = os.path.join(self.dataroot, "log")
		start_time = time.time()
		if self.verbose:
				print(f"======\nLoading HAIS-bot tables...  \n - dataset= {self.version} \n - root= {self.dataroot}")

		# Explicitly assign tables to help the IDE determine valid class members.
		self.category = self.__load_table__('category')
		self.attribute = self.__load_table__('attribute')
		self.visibility = self.__load_table__('visibility')
		self.instance = self.__load_table__('instance')
		self.sensor = self.__load_table__('sensor')
		self.calibrated_sensor = self.__load_table__('calibrated_sensor')
		self.ego_pose = self.__load_table__('ego_pose')
		self.log = self.__load_table__('log')
		self.scene = self.__load_table__('scene')
		self.sample = self.__load_table__('sample')
		self.sample_data = self.__load_table__('sample_data')
		self.sample_annotation = self.__load_table__('sample_annotation')
		self.map = self.__load_table__('map')
		self.labels_config = self.__load_table__('labels_config')
		lidar_tasks = [t for t in ['lidarseg', 'panoptic'] if osp.exists(osp.join(self.table_root, t + '.json'))]
		if len(lidar_tasks) > 0:
				self.lidarseg_idx2name_mapping = dict()
				self.lidarseg_name2idx_mapping = dict()
				self.load_lidarseg_cat_name_mapping()
		for i, lidar_task in enumerate(lidar_tasks):
				if self.verbose:
						print(f'Loading HAIS-Bot-{lidar_task}...')
				if lidar_task == 'lidarseg':
						self.lidarseg = self.__load_table__(lidar_task)
				else:
						self.panoptic = self.__load_table__(lidar_task)
				setattr(self, lidar_task, self.__load_table__(lidar_task))
				label_files = os.listdir(os.path.join(self.dataroot, lidar_task, self.version))
				num_label_files = len([name for name in label_files if (name.endswith('.bin') or name.endswith('.npz'))])
				num_lidarseg_recs = len(getattr(self, lidar_task))
				assert num_lidarseg_recs == num_label_files, \
						f'Error: there are {num_label_files} label files but {num_lidarseg_recs} {lidar_task} records.'
				self.table_names.append(lidar_task)
				# Sort the colormap to ensure that it is ordered according to the indices in self.category.
				self.colormap = dict({c['name']: self.colormap[c['name']]
															for c in sorted(self.category, key=lambda k: k['index'])})

		# If available, also load the image_annotations table created by export_2d_annotations_as_json().
		if osp.exists(osp.join(self.table_root, 'image_annotations.json')):
				self.image_annotations = self.__load_table__('image_annotations')

		if self.verbose:
				for table in self.table_names:
						print("{} {},".format(len(getattr(self, table)), table))
				print("Done loading in {:.3f} seconds.\n======".format(time.time() - start_time))

	def create_database_folder_structure(self):
		import shutil
		DIR=osp.join(self.dataroot, self.version)
		inspect_file=osp.join(self.dataroot, 'inspection_dic.json')
		# removing the inspection dict
		if os.path.exists(inspect_file):
			os.remove(inspect_file)

		# removing the version folder
		if not os.path.exists(DIR):
				os.makedirs(DIR)
		else:
			shutil.rmtree(DIR) 
			os.makedirs(DIR)
			print(f'\n Warrning: the old {self.version} database is removed and replaced by a new one!! \n')

	def pkl_to_dict(self, filename):
		open_file = open(filename, "rb")
		dict = pickle.load(open_file)
		open_file.close()
		return dict

	def get_table_path(self, table_name):
		return osp.join(self.table_root, '{}.json'.format(table_name)) 

	def load_lidarseg_cat_name_mapping(self):
		""" Create mapping from class index to class name, and vice versa, for easy lookup later on """
		for lidarseg_category in self.category:
			# Check that the category records contain both the keys 'name' and 'index'.
			assert 'index' in lidarseg_category.keys(), \
					'Please use the category.json that comes with nuScenes-lidarseg, and not the old category.json.'

			self.lidarseg_idx2name_mapping[lidarseg_category['index']] = lidarseg_category['name']
			self.lidarseg_name2idx_mapping[lidarseg_category['name']] = lidarseg_category['index']

	def check_gps_position_format(self, translation):
		'''
		Checking the position in decimal degrees format and range from -90 to 90 for latitude and -180 to 180 for longitude.
		'''
		# latitude
		if np.abs( float(translation[0]) )>90:
			translation[0]=translation[0]/100

		# longitude
		if np.abs( float(translation[1]) )>180:
			translation[1]=translation[1]/100	

	def build_database_tables(self):
		'''
		Build Nuscenes-like DB
		'''
		# check the mission file 		
		if not os.path.exists(self.scenes_path):
			if os.path.exists(self.log_path):
				# convert the log file t a mission file for Jetson Nano node
				self.convert_JetsonNano_database(self.dataroot)
				a=1
			else:
				msg=f'\n\n - The sensors mission file [{ self.scenes_path}] and log file [{self.log_path}] do not  exist!! \
					\n Please check the file <config/config.json> config[DATA]'
				print(msg)
				raise Exception(msg)
			# return 
		# Build log table + new entry
		scene_files_list=glob(os.path.join(self.scenes_path, "*.json")) 
		print(f'- Building the Nuscenes data of the following missions: \n {scene_files_list}')
		log_filename = self.get_table_path('log')
		log_table = self.load_json(log_filename)
		log_token = self.get_log_token(self.config)
		log_ = self.build_log_table_entry(token=log_token,vehicle=self.config['vehicle'], location=self.config['location'])
		self.update_table(log_table, log_, log_filename)
		
		# Load the inspection table
		inspect_filename=osp.join(self.dataroot, 'inspection_dic.json')
		if os.path.exists(inspect_filename):
			try:
				inspect_table = self.load_json(inspect_filename)
			except Exception as e:
				print(f'\t -  Exception {e}.  \n\t    A new Json file will be created!')
				return 1
		else:
			# create an empty inspection table
			print(f' flag: create an empty inspection table: inspection_dic.json ')
			inspect_table={	'lon': [], 'lat':	[], 'alt':	[], 
								'token':[], 'road': [], 'lanemarker': [],
								'kinematics': [], 'weather': [], 'metric': []}

		# start the mission files loading/processing
		# print(f'flag: inspect_table={inspect_table}')

		# process the collected data
		for scene_id, scene_file in enumerate(scene_files_list):
			# prepaer Scene table + new entry 
			scene_data = self.load_json(scene_file)
			scene_data=[k for k in scene_data if k != {}]
			scene_filename = self.get_table_path('scene')
			scene_table = self.load_json(scene_filename)
			scene_token = self.get_scene_token(scene_file )
			scene_name = f"{self.node_name}-{os.path.basename(scene_file)[:-5]}"
			##Build sample related tables + new entry
			print(f'\n - Building the database structure of scene ({scene_name}) [{scene_id+1}/{len(scene_files_list)}]. Please wait ...')
			sample_filename = self.get_table_path('sample') 
			prev_sample=''
			err_sample=0
			sensor_scene_flag=0
			current_scene=0
			for id, sample in enumerate(tqdm(scene_data)):
				if sample!={}:
					# get sensor data
					try:
						filepath= osp.join(self.dataroot, sample['filename'])
						if not os.path.exists(filepath):
							print(f'- Error: file skipped because it is not found: {filepath}')
							continue

						sensor_name= sample["sensor_name"] 
						if sensor_scene_flag ==0:
							SCENE_sensor_name=sensor_name
							sensor_scene_flag=1

					except Exception as e:
						err_sample+=1
						# print(f' flag: sample = {sample} \n - err_sample={err_sample}')
						continue

					# get the scene number
					if sensor_name==SCENE_sensor_name:
						current_scene+=1
					# try:
					# 	current_scene=sample["scene"]
					# except:
					# 	current_scene=sample["frame"]


					timestamp = int(float(sample['timestamp'])*1000000)
					rotation, translation =sample["position"]["Rotation"], sample["position"]["Translation"]
					self.check_gps_position_format(translation)
					# input(f'flag: translation={translation}')

					is_key_frame = True
					sensor_channel= sensor_name
					sensor_modality, fileformat = self.get_sensor_modality(sensor_name), sample['fileformat'] #
					sensor_filename= sample['filename'] #os.path.join('sweeps', sensor_channel, sample_token+'.'+fileformat)
					height, width = 0, 0
					meta_data=sample['meta_data']
					# input(f'\n flag: sensor_filename={sensor_filename} , current_scene={current_scene}')
					# sample details
					sample_token = self.get_sample_token(scene_name=scene_name, scene_nb=current_scene)

					#  check  if the sample is the last?
					if id<len(scene_data)-1:
						next_sample = self.get_sample_token(scene_name=scene_name, scene_nb=current_scene+1)
					else: 
						sample_token = next_sample
						next_sample=''
						

					#  check  if the sample is the first?
					if prev_sample=='': 
						first_sample_token = sample_token

					prev_sample_data = self.get_sample_data_token(sample_token=prev_sample, frame_count=id-1)
					sample_data_token= self.get_sample_data_token(sample_token=sample_token, frame_count=id)
					next_sample_data = self.get_sample_data_token(sample_token=next_sample, frame_count=id+1)
					
					# sample_token = self.get_sample_token(scene_name=scene_name, scene_nb=current_scene, step=0)
					# prev_sample_data = self.get_sample_data_token(sample_token=sample_token, frame_count=id-1)
					# next_sample_data = self.get_sample_data_token(sample_token=next_sample, frame_count=id+1)
					
		
					# update sample data table + new entry
					sample_data_filename = self.get_table_path('sample_data')
					sample_data_table = self.load_json(sample_data_filename)
					

					# create sample data table
					calibrated_sensor_token=self.get_calib_token(sample_data_token)
					ego_pose_token=sample_data_token#elf.get_ego_pose_token(sample_token) 
					self.create_new_folder( os.path.dirname(os.path.join(self.dataroot,sensor_filename)) )
					sample_data_ = self.build_sample_data_table_entry(token=sample_data_token,timestamp=timestamp, sample_token=sample_token, ego_pose_token=ego_pose_token,\
																calibrated_sensor_token=calibrated_sensor_token,fileformat=fileformat, is_key_frame=is_key_frame,\
																		prev_sample=prev_sample_data, next_sample=next_sample_data, filename=sensor_filename, height=height, width=width, meta_data=meta_data)
					
					self.update_table(sample_data_table, sample_data_, sample_data_filename)
					
					# Update the ego-pos table
					ego_pose_filename = self.get_table_path('ego_pose')
					ego_pose_table = self.load_json(ego_pose_filename)
					ego_pose_ = self.build_ego_pose_table_entry(token=ego_pose_token, timestamp=timestamp, rotation=rotation, translation=translation)
					self.update_table(ego_pose_table, ego_pose_, ego_pose_filename)
					
					# Update the sensor table
					sensor_token = self.get_sensor_token(sensor_name, self.config)
					sensor_filename = self.get_table_path('sensor')
					sensor_table = self.load_json(sensor_filename)
					sensor_ = self.build_sensor_table_entry(token=sensor_token, channel=sensor_channel, modality=sensor_modality)
					self.update_table(sensor_table, sensor_, sensor_filename)
					
					# Update the calibrated sensor table
					rotation_calib, translation_calib, camera_intrinsic = [], [], []
					calib_filename = self.get_table_path('calibrated_sensor')
					calib_table = self.load_json(calib_filename)
					calib_ = self.build_calib_table_entry(token=calibrated_sensor_token, sensor_token=sensor_token,  translation=translation_calib, \
																								rotation=rotation_calib, camera_intrinsic=camera_intrinsic)
					self.update_table(calib_table, calib_, calib_filename)

					# inspection table
					if not sample_token in inspect_table['token']:
						road_metric = 0 #random.randint(0, 4)
						inspect_table['lat'].append(translation[0])
						inspect_table['lon'].append(translation[1])
						inspect_table['alt'].append(translation[2])
						inspect_table['token'].append(sample_token)
						inspect_table['road'].append(road_metric)
						inspect_table['lanemarker'].append(road_metric)
						inspect_table['kinematics'].append(road_metric)
						inspect_table['weather'].append(road_metric)
						inspect_table['metric'].append(road_metric)

					# update sample tabel?
					# print(f'\n flag: \n - prev_sample={prev_sample}\n - sample_token={sample_token}\n - next_sample={next_sample}')
					sample_ = self.build_sample_table_entry(token=sample_token,timestamp=timestamp,prev_sample=prev_sample, \
																										next_sample=next_sample, scene_token=scene_token)
					sample_table = self.load_json(sample_filename)
					self.update_table(sample_table, sample_, sample_filename)

					# moving to next sample
					prev_sample=sample_token
					sample_token=next_sample
					# old_scene+=1

			# Build Scene table + new entry 
			last_sample_token = prev_sample
			scene_ = self.build_scene_table_entry(name=scene_name, token=scene_token,log_token=log_token,\
																							scene_table=scene_data, first_sample_token=first_sample_token,\
																							last_sample_token=last_sample_token, description=self.config['description'])
			# print(f'\n\n  -> scene_:\n{scene_}')
			self.update_table(scene_table, scene_, scene_filename)

		# save inspection table
		self.save_json(inspect_table, inspect_filename)
		# Default Table: maps, etc
		self.create_default_tables(log_tokens=[log_token])
		# display
		print(f'\n\n  # The Generated table are saved the following directory: \n {self.table_root}')
		return 0

	def create_default_tables(self, log_tokens):
		'''
		Build the default tables
		'''
		# Build category + new entry 
		category_filename = self.get_table_path('category')
		category_table = self.load_json(category_filename)
		category_ =[
								{
									"token": "1",
									"name": "road.public.Concrete.Bus_lane",
									"description": "Bus lane in Concrete."
								},
								{
									"token": "2",
									"name": "road.public.bridge",
									"description": "bridge."
								}
							]
		self.save_json(category_, category_filename)
		# self.update_table(category_table, category_, category_filename)

		# Build maps + new entry 
		maps_filename = self.get_table_path('map')
		maps_table = self.load_json(maps_filename)
		maps_ =[
							{
								"category": "1",
								"token": "1",
								"filename": "maps/map1.png",
								"log_tokens": log_tokens
							}
						]
		self.save_json(maps_, maps_filename)
		# self.update_table(maps_table, maps_, maps_filename)

		# copy map image
		import shutil
		src="bin/maps/map1.png" 
		dst=os.path.join( self.dataroot, maps_[0]["filename"] )
		self.create_new_folder(os.path.dirname(dst))
		shutil.copy(src, dst)
		print( f'\n info: map is copied in {dst}')

	def annotate_database(self,scene):
		'''
		Annotate the collected measurements
		'''
		# nuscene manager
		from nuscenes import NuScenes
		nusc = NuScenes(version=self.version, dataroot=self.dataroot, verbose=False)
		## start the annotation
		scene_token=scene['token']
		print(f'\n########################################################')
		print(f'\n##                 Scene annotation                   ##')
		print(f'\n## - token={scene_token}                                   ')
		print(  f'########################################################')
		# Update the instance table
		instance_token = self.get_instance_token(scene_token)
		instance_filename = self.get_table_path('instance')
		instance_table = self.load_json(instance_filename)
		category_token='1'
		nbr_annotations=0
		first_annotation_token= self.get_sample_annotation_token(scene['first_sample_token'])
		last_annotation_token = self.get_sample_annotation_token(scene['last_sample_token'])

		instance_ = self.build_instance_table_entry(token=instance_token, category_token=category_token, nbr_annotations=nbr_annotations,\
																		first_annotation_token=first_annotation_token, last_annotation_token=last_annotation_token)
		self.update_table(instance_table, instance_, instance_filename)

		# Update the sample annotation table
		sample_data_token = scene['first_sample_token']
		current_sample= scene['first_sample_token']
		prev_sample = nusc.get('sample', current_sample)['prev']
		next_sample = nusc.get('sample', current_sample)['next']
		print(f'\n flag0: \n - prev_sample = {prev_sample} \n - next_sample = {next_sample}')
		input(f'\n flag: \n - curr_sample = \n{nusc.get("sample", current_sample)}')
		while current_sample!='':
			print(f'\n\n  --> annotating the sample: {current_sample}')
			next_sample = nusc.get('sample', current_sample)['next']
			sample_annotation_token = self.get_sample_annotation_token(current_sample)
			sample_annotation_filename = self.get_table_path('sample_annotation')
			sample_annotation_table = self.load_json(sample_annotation_filename)
			visibility_token='1'
			attribute_tokens=['1','2']
			translation=[33.26,10.419,0.8]
			size=[0.621,0.669,1.642]
			rotation=[0.9831098797903927,0.0,0.0,-0.18301629506281616]
			prev_sample_annotation=self.get_sample_annotation_token(prev_sample)
			next_sample_annotation=self.get_sample_annotation_token(next_sample)
			num_lidar_pts=0
			num_radar_pts=0
			sample_annotation_= self.build_sample_annotation_table_entry(token=sample_annotation_token, sample_token=sample_data_token,\
														instance_token=instance_token, visibility_token=visibility_token, attribute_tokens=attribute_tokens,\
														translation=translation, size=size, rotation=rotation, prev_sample_annotation=prev_sample_annotation,\
														next_sample_annotation=next_sample_annotation,num_lidar_pts=num_lidar_pts,num_radar_pts=num_radar_pts)
			self.update_table(sample_annotation_table, sample_annotation_, sample_annotation_filename)

			# get the next sample
			prev_sample = current_sample
			current_sample = next_sample

	#################	 BASIC FUNCTIONS	#################
	def get_table_size(self, filename):
		try:
			if os.path.exists(filename):
				import json
				f = open(filename,)
				data = json.load(f)
				f.close()
			else:
				data=[]

			return len(data)+1
		except Exception as e:
			msg = f'\n\n Error: The JSON file <{filename}> cannot be read correctly!! \n Exception: {e}'
			print(msg)
			return 0

	def load_json(self, filename):
		
		try:
			if os.path.exists(filename):
				import json
				with open(filename) as f:
					data = json.load(f)
				f.close()
				# if len(data)==0:
				# 	print(f'\n => warning: the loaded file is empty!!! \n filepath={filename}')
				return data

			else:
				msg =f'\n\n Error: The JSON file <{filename}> cannot be found!!'
				print(msg)
				raise Exception(msg)

		except Exception as e:
			msg = f'\n\n Error: The JSON file <{filename}> cannot be read correctly!! \n Exception: {e}'
			print(msg)
			raise ValueError(msg)

	def save_json(self, json_string, filename):
		import json
		try:
			# Using a JSON string
			with open(filename, 'w') as outfile:
				json.dump(json_string, outfile, indent=2)
				return 0
		except Exception as e:
			print(f'\n\n - error in saving {filename}\n Exception: {e}')
			return 1

	def pkl_to_dict(self, filename):
		import pickle
		open_file = open(filename, "rb")
		dict = pickle.load(open_file)
		open_file.close()
		return dict

	def get_sensor_filename(self, sensor_name, frame):
		time_tag =  str(self.get_time_tag(type=1))
		return time_tag + '__' + sensor_name + '__' + str(frame)

	def get_time_tag(self, type=1):
		from datetime import datetime
		today = datetime.now()
		if type==0:
				return today.strftime("__%Y-%m-%d")
		else:
				return today.strftime("%Y-%m-%d-%Hh-%Mmin-%Ssec")

	def create_new_folder(self, DIR):
		if not os.path.exists(DIR):
			os.makedirs(DIR)

	#################	 SAMPLE TABLE	################# 
	def update_table(self,old_table, entry, filename):
		if len(old_table)>0:
			e=entry['token']
			for ID in old_table:
				if ID['token']==entry['token']:
					msg = f'\n\n Error: The token <{e}> already exist in the table < {os.path.basename(filename)[:-5]} >'
					#print(msg)
					# raise ValueError(msg)
					return 1
		table = old_table + [entry]
		self.save_json(table, filename)

	#################	 LOG TABLE	################# 
	def get_log_token(self, config):
		return config['vehicle'] +'__'+  config['location']

	def build_log_table_entry(self, token,vehicle,location):
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

	#################	 MAPS TABLE	################# 
	def get_maps_token(self, config):
		return config['vehicle'] +'__'+  config['location']

	def build_maps_table_entry(self, token,vehicle,location):
		from datetime import datetime
		today = datetime.now()
		logfile = vehicle + '_' + today.strftime("%Y-%m-%d-%Hh-%Mmin")
		date_captured=today.strftime("%Y-%m-%d-%Hh-%Mmin-%Ssec")
		entry = {
		"token": token,
		"category": logfile,
		"filename": vehicle,
		"log_tokens": date_captured,
		}
		return entry

	#################	 SCENE TABLE	 ################# 
	def get_scene_token(self, scene_file):
		return os.path.basename(scene_file)[:-5]

	def build_scene_table_entry(self, name,token,log_token,scene_table, first_sample_token, last_sample_token, description):
		nbr_samples = len(scene_table)
		entry = {
		"token": token,
		"log_token": log_token,
		"nbr_samples": nbr_samples,
		"first_sample_token": first_sample_token,
		"last_sample_token": last_sample_token,
		"name": name,
		"description": description
		}
		return entry

	#################	 SAMPLE TABLE	 ################# 
	def get_sample_token(self, scene_name, scene_nb):
		if scene_name=='':
			return ''
		scene_tag = 's'+str(scene_nb)
		return scene_name + '_' +  scene_tag

	def build_sample_table_entry(self, token,timestamp,prev_sample, next_sample, scene_token):
		entry = {
		"token": token,
		"timestamp": timestamp,
		"prev": prev_sample,
		"next": next_sample,
		"scene_token": scene_token
		}
		return entry

	#################	 SAMPLE TABLE	 ################# 
	def get_sample_data_token(self, sample_token, frame_count=0):
		if sample_token=="":
			return ""
		return sample_token+'_f'+str(frame_count)

	def build_sample_data_table_entry(self, token,timestamp, sample_token, ego_pose_token, calibrated_sensor_token, \
																	fileformat, is_key_frame, prev_sample, next_sample, filename, height=0, width=0, meta_data=""):
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
		"meta_data": meta_data,
		"prev": prev_sample,
		"next": next_sample,
		}
		return entry

	#################	 EGO-POS TABLE	 ################# 
	def build_ego_pose_table_entry(self, token, timestamp, rotation=[], translation=[]):
		entry ={
		"token": token,
		"timestamp": timestamp,
		"rotation": rotation ,
		"translation": translation
		}
		return entry

	#################	 CALIBRATION TABLE	 ################# 
	def get_calib_token(self, sample_data_token):
		return 'calib__' + sample_data_token

	def build_calib_table_entry(self, token, sensor_token,  translation=[], rotation=[], camera_intrinsic=[]):
		entry ={
		"token": token,
		"sensor_token": sensor_token,
		"translation": translation,
		"rotation": rotation,
		"camera_intrinsic":camera_intrinsic
		}
		return entry

	#################	 SENSOR TABLE	 ################# 
	def get_sensor_token(self, sensor_name, config):
		return config['vehicle'] + '__' + sensor_name

	def get_sensor_modality(self, sensor_name):
		import re
		if re.search('CAM', sensor_name, re.IGNORECASE):
			modality ="camera"

		elif re.search('LIDAR', sensor_name, re.IGNORECASE):  
			modality ="lidar"

		elif re.search('RADAR', sensor_name, re.IGNORECASE):
			modality ="radar"

		elif re.search('IMU', sensor_name, re.IGNORECASE):
			modality ="imu"

		elif re.search('GPS', sensor_name, re.IGNORECASE):
			modality ="gps"
		else:
			print(f'\n Error: The sensor name <{sensor_name}> is not defined!!!')
			modality ="Undefined"
		
		return  modality

	def build_sensor_table_entry(self, token, channel, modality):
		entry ={
				"token": token,
				"channel": channel,
				"modality": modality
				}
		return entry

	#################	 INSTANCE TABLE	 ################# 

	def get_instance_token(self, scene_token):
		return 'ann-scene__' + scene_token

	def build_instance_table_entry(self, token, category_token, nbr_annotations, first_annotation_token, last_annotation_token):
		entry ={
			"token": token,
			"category_token": category_token,
			"nbr_annotations": nbr_annotations,
			"first_annotation_token": first_annotation_token,
			"last_annotation_token": last_annotation_token
			}
		return entry

	#################	 ANNOTATION TABLE	 ################# 
	def get_sample_annotation_token(self, sample_data_token):
		if sample_data_token == '':
			return ''
		else:
			return 'ann-sdata__' + sample_data_token

	def build_sample_annotation_table_entry(self, token, sample_token, instance_token, visibility_token, attribute_tokens,\
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

#%%	#################	 Main methods	 ################# 
	def load_create_database(self):
		'''
		Creation of a structured HAIS database
		'''
		try:
			from nuscenes.nuscenes import NuScenes
			self.nusc = NuScenes(version=self.version, dataroot=self.dataroot, verbose=self.verbose)
			# my_scene=self.nusc.scene[0]
			# self.nusc.list_scenes()
			self.explore_database(verbose=False)
			build_database=False
			if self.verbose:
				print(f'\n - The Nuscenes database is already created.')
			
		except Exception as e:
			build_database=True
			print(f'\n The Nuscenes database does not exist or corrupted. \nException: {e}')
			
		# check the inspection json
		self.inspect_json_file=os.path.join(self.dataroot, 'inspection_dic.json')
		if not os.path.exists(self.inspect_json_file):
			build_database=True

		if build_database:
			# display
			if self.verbose:
				print(f'\n - Building the Nuscenes database ...')
				print(f'\t - dataset root={self.dataroot} ')
			# create the database root
			self.create_database_folder_structure()
			# If Drone, build the predefined data structure od HAIS node
			self.convert_drone_database()
			# convert the collected data to Nuscenes database 
			err=self.build_database_tables()

			# create the Nuscenes DB object
			try:
				self.nusc = NuScenes(version=self.version, dataroot=self.dataroot, verbose=True)
			except Exception as e: 
				msg=f'Error: The loaded data seems to be corrupted!! \
	 					 \n Please recheck the missions JSON files and the corresponding sensors data: \
						 \n - node= {os.path.basename(self.dataroot)}\
						 \n - {e}'
				print(msg)
				return 1, msg
		
		# load the inspection json
		self.inspection_dict=self.load_json(self.inspect_json_file)
		self.ego_idx=0
		self.ego_token=self.inspection_dict['token'][self.ego_idx]
		# self.sample_token=self.ego_token.split('_f')[0]
		self.sample_token=self.ego_token
		self.my_sample = self.nusc.get('sample', self.sample_token)
		msg =f'The data [{os.path.basename(self.dataroot)}] is structured successfully!!'
		return 0, msg


	def update_inspection_dict(self, list_metrics):
		# print(f'\n - list_metrics ={list_metrics}')
		# print(f'\n - keys = \n new={list_metrics.keys()} dict={self.inspection_dict.keys()}')
		# self.save_json(self.inspection_dict, self.inspect_json_file)
		new_token=list_metrics['token']
		if new_token!='':
			tokens=self.inspection_dict['token']
			try:
				idx=tokens.index(new_token) 
				# print(f'\n  ==> list_metrics={list_metrics} \n - new_token={new_token}')
			except Exception as e:
				# print(f'- Error: The new token=[{new_token}] does not exist in list of metrics! \
	  		# 			\n - list_metrics={list_metrics} \n - new_token={new_token}')
				return 1

			# update the correspnding metric
			for metric in list_metrics.keys():
				if metric!='token':
					try:
						self.inspection_dict[metric][idx]=list_metrics[metric]
					except Exception as e:
						print(f'\n Error: error in update the inspection metric {metric} !! \nException:{e}')

			# input(f"\n changed!! \n - inspection_dict=  {self.inspection_dict}")
			self.save_json(self.inspection_dict, self.inspect_json_file)
			return 0

	def get_IMU_kinematics(self, sample_token):
		try:
			self.sample = self.nusc.get('sample', self.sample_token)
			imu_token=self.sample['data']['IMU_SENSOR']
			imu_sample_data = self.nusc.get('sample_data', imu_token)
			self.kinematic_dict=imu_sample_data['meta_data']
			# print(f'\n flag: kinematic_dict={self.kinematic_dict}')
		except Exception as e:
			print(f'\n - error in reading the stored IMU data!!\nException:{e}')
			self.kinematic_dict={}

	def get_LIDAR(self, sample_token):
		try:
			self.sample = self.nusc.get('sample', self.sample_token)
			imu_token=self.sample['data']['IMU_SENSOR']
			imu_sample_data = self.nusc.get('sample_data', imu_token)
			self.kinematic_dict=imu_sample_data['meta_data']
			# print(f'\n flag: kinematic_dict={self.kinematic_dict}')
		except Exception as e:
			print(f'\n - error in reading the stored IMU data!!\nException:{e}')
			self.kinematic_dict={}

	def get_Road_Camera(self, sample_token):
		try:
			self.sample = self.nusc.get('sample', self.sample_token)
			imu_token=self.sample['data']['IMU_SENSOR']
			imu_sample_data = self.nusc.get('sample_data', imu_token)
			self.kinematic_dict=imu_sample_data['meta_data']
			# print(f'\n flag: kinematic_dict={self.kinematic_dict}')
		except Exception as e:
			print(f'\n - error in reading the stored IMU data!!\nException:{e}')
			self.kinematic_dict={}

	def get_Lanemarker_Camera(self, sample_token):
		try:
			self.sample = self.nusc.get('sample', self.sample_token)
			imu_token=self.sample['data']['IMU_SENSOR']
			imu_sample_data = self.nusc.get('sample_data', imu_token)
			self.kinematic_dict=imu_sample_data['meta_data']
			# print(f'\n flag: kinematic_dict={self.kinematic_dict}')
		except Exception as e:
			print(f'\n - error in reading the stored IMU data!!\nException:{e}')
			self.kinematic_dict={}

	def get_file_path_ego(self,n=0):
		try:
			self.sample_token=self.inspection_dict['token'][self.ego_idx]
			# print(f'\n ego_token={ego_token}')
		except Exception as e:
			print(f'\n error in loading the ego token [{self.ego_idx}]!!!\n Exception:{e}')
			if n==1:
				self.ego_idx=0
			else:
				self.ego_idx=-1
			return 1

		self.ego_idx+=n
		# get the sensors data
		self.sensors_data_list=self.get_sample_files_data(n=n)
		# input(f'\n self.sensors_data_list={self.sensors_data_list}')
		self.sensor_data_dict={}
		for sensor_name in self.sensors_data_list.keys():
			# get the kinematics data
			if 'IMU' in sensor_name:
				sensor_token=self.sensors_data_list[sensor_name]
				sensor_sample_data = self.nusc.get('sample_data', sensor_token)
				self.sensor_data_dict.update({sensor_name:sensor_sample_data['meta_data']})
			elif  'LIDAR' in sensor_name or 'CAMERA' in sensor_name:
				sensor_token=self.sensors_data_list[sensor_name]
				sensor_sample_data = self.nusc.get('sample_data', sensor_token)
				self.sensor_data_dict.update({sensor_name:os.path.join(self.dataroot, sensor_sample_data['filename'])})
			elif  'GPS'  in sensor_name:
				sensor_token=self.sensors_data_list[sensor_name]
				ego_pos=self.nusc.get('ego_pose', sensor_token)
				car_position=ego_pos["translation"]
				self.sensor_data_dict.update({sensor_name:car_position})
		return 0

	def get_sample_files_data(self, n=1):
		# input(f'\n self.my_sample={self.my_sample}\n sensor_name={sensor_name} ')
		if self.sample_token=='':
			# print(f'\n flag: n= {n} --> sample token ={self.sample_token} \n self.my_sample={self.my_sample}')
			return '', [-1,-1,-1]
		self.my_sample = self.nusc.get('sample', self.sample_token)
		if n==1:
			self.sample_token=self.my_sample['next']
		elif n==-1:
			self.sample_token=self.my_sample['prev']
		# input(f'\n n={n} --> self.sample_token={self.sample_token}')

		# input(f'\n self.my_sample={self.my_sample}\n sensor_name={sensor_name} ')
		sensors_list=self.my_sample['data']
		return sensors_list

	def get_file_path(self, sensor_name, n=1):
		# input(f'\n self.my_sample={self.my_sample}\n sensor_name={sensor_name} ')
		if self.sample_token=='':
			# print(f'\n flag: n= {n} --> sample token ={self.sample_token} \n self.my_sample={self.my_sample}')
			return '', [-1,-1,-1]
		self.my_sample = self.nusc.get('sample', self.sample_token)
		if n==1:
			self.sample_token=self.my_sample['next']
		elif n==-1:
			self.sample_token=self.my_sample['prev']
		# input(f'\n n={n} --> self.sample_token={self.sample_token}')

		# input(f'\n self.my_sample={self.my_sample}\n sensor_name={sensor_name} ')
		sensors_list=self.my_sample['data']
		if sensor_name in sensors_list.keys():
			sensor_data_sample=self.nusc.get('sample_data', self.my_sample['data'][sensor_name])
			filename=os.path.join(self.dataroot, sensor_data_sample['filename'])
			ego_pos=self.nusc.get('ego_pose', sensor_data_sample['ego_pose_token'])
			car_position=ego_pos["translation"]
			# input(f'\n filename={filename}\n\n ego_pos={ego_pos}')
			return filename, car_position
		else:
			return self.get_file_path(sensor_name, n=n) 
			# return '', [-1,-1,-1]

	def get_list_sensors(self):
		self.my_scene=self.nusc.scene[0]
		self.sample_token=self.my_scene['first_sample_token']
		self.my_sample = self.nusc.get('sample', self.sample_token)
		# print(f'\n\n ==> First sample of the scene: \n {my_sample}')
		sensors=self.my_sample['data']
		list_sensors=list(sensors.keys())
		return list_sensors
	
	def get_list_sensors(self):
		self.my_scene=self.nusc.scene[0]
		self.sample_token=self.my_scene['first_sample_token']
		self.my_sample = self.nusc.get('sample', self.sample_token)
		# print(f'\n\n ==> First sample of the scene: \n {my_sample}')
		sensors=self.my_sample['data']
		list_sensors=list(sensors.keys())
		return list_sensors
	
	def explore_database(self, verbose=True):

		my_scene=self.nusc.scene[0]
		if verbose:
			print(f'\n\n ==> List of recorded scenes: \n ')
		self.nusc.list_scenes()

		if verbose:
			print(f'\n\n ==> First scene: \n {my_scene}')

		my_sample = self.nusc.get('sample', my_scene['first_sample_token'])
		if verbose:
			print(f'\n\n ==> First sample of the scene: \n {my_sample}')

		sensors=my_sample['data']
		list_sensors=self.get_list_sensors()
		if verbose:
			print(f'\n\n ==> List of sensors: \n {list_sensors}')

###########################

def test_HAIS_database():
	##-------------------  NODE -------------------
	dataroot= '../data/download/node5-JN' 
	# dataroot= '../data/download/node1' 
	version='v0.0'
	verbose=True
	# load/create the database structure from the collected inspection sensors dataset
	DB = HAIS_database(dataroot=dataroot, version=version, verbose=verbose)
	# explore the database
	DB.explore_database()
	# list sensors names
	list_sensors=DB.get_list_sensors()
	files_list=[]
	cnt=0
	while True:	
	# for n in range(800):
		# filename, car_position=DB.get_file_path(sensor_name, n=1)
		err=DB.get_file_path_ego(n=1)
		# print(f'\n sensor_data_dict={DB.sensor_data_dict}')
		files_list.append(DB.sensor_data_dict)
		cnt+=1
		if err==1 or DB.sensor_data_dict=={}:
			break
	return 0

if __name__ == '__main__':
	# test database
	DB=test_HAIS_database()
	
  
	



 
