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
from lib import utils

class HAIS_bot:
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
				self.config=utils.load_json(config)
				self.version = version
				self.dataroot = dataroot
				self.verbose = verbose
				self.table_names = ['category', 'attribute', 'visibility', 'instance', 'sensor', 'calibrated_sensor',
														'ego_pose', 'log', 'scene', 'sample', 'sample_data', 'sample_annotation', 'map', 'labels_config']

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
					utils.create_new_folder(osp.dirname(filename))
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
			list_sensors, init_sensors={}, {}
			# Build log table + new entry 
			log_filename = self.get_table_path('log')
			log_table = utils.load_json(log_filename)
			log_token = utils.get_log_token(self.config)
			log_ = utils.build_log_table_entry(token=log_token,vehicle=self.config['vehicle'], location=self.config['location'])
			utils.update_table(log_table, log_, log_filename)

			# prepaer Scene table + new entry 
			scene_data = self.pkl_to_dict(self.config['filename'])[:17]
			# print(f'\n\n  -> scene_data :\n{scene_data}')
			scene_filename = self.get_table_path('scene')
			scene_table = utils.load_json(scene_filename)
			scene_token = utils.get_scene_token(config=self.config)
			# print(f'\n\n  -> scene_token:\n{scene_token}')
			scene_name = self.config['Scenario']+ '-' + self.config['USE_CASE']

			##Build sample related tables + new entry 
			sample_filename = self.get_table_path('sample') 
			prev_sample=''
			sensor_list=[]
			for id , sample in enumerate(scene_data):
				# get sensor data
				sensor_name=list(sample.keys())[1]
				if not sensor_name in list_sensors.keys():
					list_sensors.update({sensor_name:0})
					init_sensors.update({sensor_name:0})
				# get the timestamp data
				timestamp = sample[sensor_name]['Timestamp']

				# confirm sensor sample update 
				list_sensors[sensor_name]=list_sensors[sensor_name]+1
				#  check  if the sample is the last?
				try:
					next_sample = utils.get_sample_token(scene_data[id+1], config=self.config)
				except:
					next_sample=''
				#  check  if the sample is the first?
				if prev_sample=='':
					sample_token = utils.get_sample_token(sample, config=self.config)
					first_sample_token = sample_token
				# update sample data table + new entry
				sample_data_filename = self.get_table_path('sample_data')
				sample_data_table = utils.load_json(sample_data_filename)
				calibrated_sensor_token=utils.get_calib_token(sample_token)
				ego_pose_token=utils.get_ego_pose_token(sample_token) 
				if list_sensors[sensor_name]<=1:
					is_key_frame = False #True 
				else:
					is_key_frame = False
				sample_data_token= utils.get_sample_data_token(sample_token=sample_token, list_sensors=list_sensors) 
				prev_sample_data = utils.get_sample_data_token(sample_token=prev_sample, list_sensors=list_sensors)
				next_sample_data = utils.get_sample_data_token(sample_token=next_sample, list_sensors=list_sensors)
				sensor_channel= sensor_name
				sensor_modality, fileformat = utils.get_sensor_modality(sensor_name)
				sensor_filename= os.path.join('sweeps', sensor_channel, sample_token+'.'+fileformat)
				height, width = 0, 0
				# create data folder
				print(os.path.dirname(os.path.join(self.dataroot,sensor_filename)) )
				utils.create_new_folder( os.path.dirname(os.path.join(self.dataroot,sensor_filename)) )
				sample_data_ = utils.build_sample_data_table_entry(token=sample_data_token,timestamp=timestamp, sample_token=sample_token, ego_pose_token=ego_pose_token,\
															calibrated_sensor_token=calibrated_sensor_token,fileformat=fileformat, is_key_frame=is_key_frame,\
																	prev_sample=prev_sample_data, next_sample=next_sample_data, filename=sensor_filename, height=height, width=width)

				utils.update_table(sample_data_table, sample_data_, sample_data_filename)

				# Update the ego-pos table
				rotation, translation = [], []
				ego_pose_filename = self.get_table_path('ego_pose')
				ego_pose_table = utils.load_json(ego_pose_filename)
				ego_pose_ = utils.build_ego_pose_table_entry(token=ego_pose_token, timestamp=timestamp, rotation=rotation, translation=translation)
				utils.update_table(ego_pose_table, ego_pose_, ego_pose_filename)
				
				# Update the sensor table
				sensor_token = utils.get_sensor_token(sensor_name, self.config)
				sensor_filename = self.get_table_path('sensor')
				sensor_table = utils.load_json(sensor_filename)
				sensor_ = utils.build_sensor_table_entry(token=sensor_token, channel=sensor_channel, modality=sensor_modality)
				utils.update_table(sensor_table, sensor_, sensor_filename)
				
				# Update the calibrated sensor table
				rotation_calib, translation_calib, camera_intrinsic = [], [], []
				calib_filename = self.get_table_path('calibrated_sensor')
				calib_table = utils.load_json(calib_filename)
				calib_ = utils.build_calib_table_entry(token=calibrated_sensor_token, sensor_token=sensor_token,  translation=translation_calib, \
																							rotation=rotation_calib, camera_intrinsic=camera_intrinsic)
				utils.update_table(calib_table, calib_, calib_filename)
				
				#  check  if a new sample of measurment is presented?
				new_sample_flag = all([list_sensors[v]>0 for v in list_sensors])
				if prev_sample=='' or next_sample=='' or new_sample_flag:
					# build sample table
					# input(f'\n flag: \n - prev_sample={prev_sample}\n - sample_token={sample_token}\n - next_sample={next_sample}')
					sample_ = utils.build_sample_table_entry(token=sample_token,timestamp=timestamp,prev_sample=prev_sample, \
																										next_sample=next_sample, scene_token=scene_token)
					sample_table = utils.load_json(sample_filename)
					utils.update_table(sample_table, sample_, sample_filename)

					# moving to next sample
					prev_sample=sample_token
					sample_token=next_sample
					list_sensors = init_sensors.copy()
				
			# 		sensor_list.append(sensor_name)
			# print(np.unique(sensor_list))

			# Build Scene table + new entry 
			last_sample_token = prev_sample
			scene_ = utils.build_scene_table_entry(name=scene_name, token=scene_token,log_token=log_token,\
																							scene_table=scene_data, first_sample_token=first_sample_token,\
																					 		last_sample_token=last_sample_token, config=self.config)
			# print(f'\n\n  -> scene_:\n{scene_}')
			utils.update_table(scene_table, scene_, scene_filename)
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
			instance_token = utils.get_instance_token(scene_token)
			instance_filename = self.get_table_path('instance')
			instance_table = utils.load_json(instance_filename)
			category_token='1'
			nbr_annotations=0
			first_annotation_token= utils.get_sample_annotation_token(scene['first_sample_token'])
			last_annotation_token = utils.get_sample_annotation_token(scene['last_sample_token'])

			instance_ = utils.build_instance_table_entry(token=instance_token, category_token=category_token, nbr_annotations=nbr_annotations,\
																			first_annotation_token=first_annotation_token, last_annotation_token=last_annotation_token)
			utils.update_table(instance_table, instance_, instance_filename)

			# Update the sample annotation table
			sample_data_token = scene['first_sample_token']
			current_sample= scene['first_sample_token']
			prev_sample = nusc.get('sample', current_sample)['prev']
			next_sample = nusc.get('sample', current_sample)['next']
			# print(f'\n flag0: \n - prev_sample = {prev_sample} \n - next_sample = {next_sample}')

			# input(f'\n flag: \n - curr_sample = \n{nusc.get("sample", current_sample)}')
			while current_sample!='':
				next_sample = nusc.get('sample', current_sample)['next']
				sample_annotation_token = utils.get_sample_annotation_token(current_sample)
				sample_annotation_filename = self.get_table_path('sample_annotation')
				sample_annotation_table = utils.load_json(sample_annotation_filename)
				visibility_token='1'
				attribute_tokens=['1','2']
				translation=[33.26,10.419,0.8]
				size=[0.621,0.669,1.642]
				rotation=[0.9831098797903927,0.0,0.0,-0.18301629506281616]
				prev_sample_annotation=utils.get_sample_annotation_token(prev_sample)
				next_sample_annotation=utils.get_sample_annotation_token(next_sample)
				num_lidar_pts=0
				num_radar_pts=0
				sample_annotation_= utils.build_sample_annotation_table_entry(token=sample_annotation_token, sample_token=sample_data_token,\
															instance_token=instance_token, visibility_token=visibility_token, attribute_tokens=attribute_tokens,\
															translation=translation, size=size, rotation=rotation, prev_sample_annotation=prev_sample_annotation,\
															next_sample_annotation=next_sample_annotation,num_lidar_pts=num_lidar_pts,num_radar_pts=num_radar_pts)
				utils.update_table(sample_annotation_table, sample_annotation_, sample_annotation_filename)

				# get the next sample
				prev_sample = current_sample
				current_sample = next_sample
			