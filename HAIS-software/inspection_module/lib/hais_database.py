# HAIS-Bot.
# Code written by Abderrazak Chahid, 2022.
# NB: some part/structure are inspired/used from nuScenes
# code source: https://github.com/nutonomy/nuscenes-devkit/blob/master/python-sdk/nuscenes/nuscenes.py

import os
import time
import json
import pickle
import os.path as osp
import numpy as np

class HAIS_database:
	"""
	Database class for HAIS-bot to store compatible data structue with the nuScenes database structure.
	"""

	def __init__(self,
								config: str = 'config.json',
								version: str = 'v0.1',
								dataroot: str = './data/',
								verbose: bool = True,
								map_resolution: float = 0.1):
			"""
			Loads/Collected data and saved the json tables and filder/files strucutre 
			:param config: Collected data setup configuration: inspection node, desctiption,etc.
			:param version: Version to load (e.g. "v1.0", ...).
			:param dataroot: Path to the raw data dictionnary.
			:param verbose: Whether to print status messages during load.
			:param map_resolution: Resolution of maps (meters).
			"""
			self.config=self.load_json(config)
			if self.config["version"]!="":
				self.version = self.config["version"]
			else:
				self.version = version
			self.dataroot = dataroot
			self.verbose = verbose
			self.table_names = ['category', 'attribute', 'visibility', 'instance', 'sensor', 'calibrated_sensor',
													'ego_pose', 'log', 'scene', 'sample', 'sample_data', 'sample_annotation', 'map', 'labels_config']
			self.sensor_data_filename = os.path.join(self.config["raw_data"], self.config["json_filename"])
			start_time = time.time()
			if verbose:
					print(f"======\nLoading HAIS-bot tables...  \n - dataset= {self.version}")

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

			if verbose:
					for table in self.table_names:
							print("{} {},".format(len(getattr(self, table)), table))
					print("Done loading in {:.3f} seconds.\n======".format(time.time() - start_time))


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

	def create_database(self):
		print(f'\n########################################################')
		print(f'\n##            Updating the HAIS database              ##')
		print(f'\n## - Sensor file={self.sensor_data_filename}                                   ')
		print(  f'########################################################')
		if not os.path.exists(self.sensor_data_filename):
			msg=f'\n\n - The sensors data file [{self.sensor_data_filename}] does not  exist!! \
				\n Please check the confif/config.json file. '
			raise Exception(msg)
		# initialization
		list_sensors, init_sensors={}, {}
		# Build log table + new entry 
		log_filename = self.get_table_path('log')
		log_table = self.load_json(log_filename)
		log_token = self.get_log_token(self.config)
		log_ = self.build_log_table_entry(token=log_token,vehicle=self.config['vehicle'], location=self.config['location'])
		self.update_table(log_table, log_, log_filename)

		# prepaer Scene table + new entry 
		scene_data = self.load_json(self.sensor_data_filename)
		# print(f'\n\n  -> scene_data :\n{scene_data}')
		scene_filename = self.get_table_path('scene')
		scene_table = self.load_json(scene_filename)
		scene_token = self.get_scene_token(config=self.config)
		# print(f'\n\n  -> scene_token:\n{scene_token}')
		scene_name = self.config['Scenario']+ '-' + self.config['USE_CASE']

		##Build sample related tables + new entry 
		sample_filename = self.get_table_path('sample') 
		prev_sample=''
		sensor_list=[]
		for id , sample in enumerate(scene_data):
			if sample!={}:
				# get sensor data
				sensor_name= sample["sensor_name"] # list(sample.keys())[1]
				if not sensor_name in list_sensors.keys():
					list_sensors.update({sensor_name:0})
					init_sensors.update({sensor_name:0})
				# get the timestamp data
				timestamp = sample['timestamp']

				# confirm sensor sample update 
				list_sensors[sensor_name]=list_sensors[sensor_name]+1
				#  check  if the sample is the last?
				try:
					next_sample = self.get_sample_token(scene_data[id+1], config=self.config)
				except:
					next_sample=''
				#  check  if the sample is the first?
				if prev_sample=='':
					sample_token = self.get_sample_token(sample, config=self.config)
					first_sample_token = sample_token
				# update sample data table + new entry
				sample_data_filename = self.get_table_path('sample_data')
				sample_data_table = self.load_json(sample_data_filename)
				calibrated_sensor_token=self.get_calib_token(sample_token)
				ego_pose_token=self.get_ego_pose_token(sample_token) 
				if list_sensors[sensor_name]<=1:
					is_key_frame = False #True 
				else:
					is_key_frame = False
				sample_data_token= self.get_sample_data_token(sample_token=sample_token, list_sensors=list_sensors) 
				prev_sample_data = self.get_sample_data_token(sample_token=prev_sample, list_sensors=list_sensors)
				next_sample_data = self.get_sample_data_token(sample_token=next_sample, list_sensors=list_sensors)
				sensor_channel= sensor_name
				sensor_modality, fileformat = self.get_sensor_modality(sensor_name)
				sensor_filename= os.path.join('sweeps', sensor_channel, sample_token+'.'+fileformat)
				height, width = 0, 0
				# create data folder
				print(os.path.dirname(os.path.join(self.dataroot,sensor_filename)) )
				self.create_new_folder( os.path.dirname(os.path.join(self.dataroot,sensor_filename)) )
				sample_data_ = self.build_sample_data_table_entry(token=sample_data_token,timestamp=timestamp, sample_token=sample_token, ego_pose_token=ego_pose_token,\
															calibrated_sensor_token=calibrated_sensor_token,fileformat=fileformat, is_key_frame=is_key_frame,\
																	prev_sample=prev_sample_data, next_sample=next_sample_data, filename=sensor_filename, height=height, width=width)

				self.update_table(sample_data_table, sample_data_, sample_data_filename)

				# Update the ego-pos table
				rotation, translation = [], []
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
				
				#  check  if a new sample of measurment is presented?
				new_sample_flag = all([list_sensors[v]>0 for v in list_sensors])
				if prev_sample=='' or next_sample=='' or new_sample_flag:
					# build sample table
					# input(f'\n flag: \n - prev_sample={prev_sample}\n - sample_token={sample_token}\n - next_sample={next_sample}')
					sample_ = self.build_sample_table_entry(token=sample_token,timestamp=timestamp,prev_sample=prev_sample, \
																										next_sample=next_sample, scene_token=scene_token)
					sample_table = self.load_json(sample_filename)
					self.update_table(sample_table, sample_, sample_filename)

					# moving to next sample
					prev_sample=sample_token
					sample_token=next_sample
					list_sensors = init_sensors.copy()
				
		# 		sensor_list.append(sensor_name)
		# print(np.unique(sensor_list))

		# Build Scene table + new entry 
		last_sample_token = prev_sample
		scene_ = self.build_scene_table_entry(name=scene_name, token=scene_token,log_token=log_token,\
																						scene_table=scene_data, first_sample_token=first_sample_token,\
																						last_sample_token=last_sample_token, description=self.config['description'])
		# print(f'\n\n  -> scene_:\n{scene_}')
		self.update_table(scene_table, scene_, scene_filename)
		# display
		print(f'\n\n  # The Generatd table are saved the follwing directory: \n {self.dataroot}')

	def annotate_database(self,scene):
		'''
		nnotate the collected measurments
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
		# print(f'\n flag0: \n - prev_sample = {prev_sample} \n - next_sample = {next_sample}')

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
	def load_json(self, filename):
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

	def save_json(self, json_string, filename):
		import json
		try:
			# Using a JSON string
			with open(filename, 'w') as outfile:
				json.dump(json_string, outfile,indent=2)
				return 0
		except:
			print(f'\n\n - error in saving {filename}')
			return 1

	def pkl_to_dict(self, filename):
		import pickle
		open_file = open(filename, "rb")
		dict = pickle.load(open_file)
		open_file.close()
		return dict

	def get_sensor_filename(self, sensor_name, frame, config):
		try:
			scene_setup = '__' + config['Scenario']+ '-' + config['USE_CASE']
		except:
			scene_setup = '__Unknown'
		time_tag =  str(self.get_time_tag(type=1))
		return time_tag + scene_setup + '__' + sensor_name + '__' + str(frame)

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
		e=entry['token']
		for ID in old_table:
			if ID['token']==entry['token']:
				msg = f'\n\n Error: The token <{e}> already exist in the table'
				print(msg)
				# raise ValueError(msg)
				return 1
		table = old_table + [entry]
		self.save_json(table, filename)

	def get_sample_token(self):
		return 0

	#################	 LOG TABLE	################# 
	def get_log_token(self, config):
		return config['vehicle'] +'__'+  config['location'] +'__'+ config['Scenario']

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

	#################	 SCENE TABLE	 ################# 
	def get_scene_token(self, config=''):
		try:
			scene_setup = '__' + config['Scenario']+ '-' + config['USE_CASE']
		except:
			scene_setup = ''
		
		time_tag = self.get_time_tag(type=1)
		return time_tag + scene_setup

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
	def get_sample_token(self, scene_, config=''):
		try:
			scene_setup = '__' + config['Scenario']+ '-' + config['USE_CASE']
		except:
			scene_setup = '__Unknown'
		time_tag = str(scene_['frame'])
		return time_tag + scene_setup

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
	def get_sample_data_token(self, sample_token, list_sensors):
		if sample_token=='':
			return ''
		else:
			frame_tag = 'sdata__' + sample_token
			sensor_tag = '__sensors' + str(np.sum([list_sensors[v]>0 for v in list_sensors]))
			return frame_tag + sensor_tag

	def build_sample_data_table_entry(self, token,timestamp, sample_token, ego_pose_token, calibrated_sensor_token, \
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
	def get_ego_pose_token(self, sample_token):
		return 'pos__' + sample_token

	def build_ego_pose_table_entry(self, token, timestamp, rotation=[], translation=[]):
		entry ={
		"token": token,
		"timestamp": timestamp,
		"rotation": rotation ,
		"translation": translation
		}
		return entry

	#################	 CLIBRATIO TABLE	 ################# 
	def get_calib_token(self, sample_token):
		return 'calib__' + sample_token

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

if __name__ == '__main__':
	# # sntax
	#  syntax()
	# collected data input
	data = HAIS_database(config='config/config.json')

	# create the database structure
	data.create_database()

	# # annotate the a scene
	# scene_=data.scene
	# print(f'\n\n - scene= {scene_}')
	# data.annotate_database(scene=scene_[0])	