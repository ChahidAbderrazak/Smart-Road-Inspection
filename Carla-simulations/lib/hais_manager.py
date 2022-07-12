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
														'ego_pose', 'log', 'scene', 'sample', 'sample_data', 'sample_annotation', 'map']

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
			# Build log table + new entry 
			log_filename = self.get_table_path('log')
			log_table = utils.load_json(log_filename)
			log_token = utils.get_log_token(log_table)
			# log_ = utils.build_log_table_entry(log_token,vehicle=config['vehicle'], location=config['location'])
			# print(f'\n\n  -> log_:\n{log_}')
			# utils.update_table(log_table, log_, log_filename)
			# print(f'\n\n  -> log_table:\n{log_table}')

			# Build Scene table + new entry 
			
			scene_data = self.pkl_to_dict(self.config['filename'])[:17]
			# print(f'\n\n  -> scene_data :\n{scene_data}')
			scene_filename = self.get_table_path('scene')
			scene_table = utils.load_json(scene_filename)
			scene_token = utils.get_scene_token(config=self.config)
			# print(f'\n\n  -> scene_token:\n{scene_token}')
			scene_name = self.config['Scenario']+ '-' + self.config['Used_Case']
			scene_ = utils.build_scene_table_entry(name=scene_name, token=scene_token,log_token=log_token,\
																						scene_table=scene_data, config=self.config)
			# print(f'\n\n  -> scene_:\n{scene_}')
			utils.update_table(scene_table, scene_, scene_filename)

			##Build sample table + new entry 
			sample_filename = self.get_table_path('sample') 
			prev_sample=''
			sensor_list=[]
			for id , sample in enumerate(scene_data):
					# get sensor data
					sensor_name=list(sample.keys())[1]
					# print(f'\n\n  -> sensor = {sensor_name}')
					token = utils.get_sample_token(sample, config=self.config)
					timestamp = sample[sensor_name]['Timestamp']
					# get previous and next sample
					try:
						next_sample = utils.get_sample_token(scene_data[id+1], config=self.config)
					except:
						next_sample=''

					# build sample table
					sample_ = utils.build_sample_table_entry(token,timestamp,prev_sample, next_sample, scene_token)
					# print(f'\n\n  -> sample_:\n{sample_}')
					sample_table = utils.load_json(sample_filename)
					utils.update_table(sample_table, sample_, sample_filename)

					# update sample data table + new entry
					prev_sample=token

					sensor_list.append(sensor_name)
			print(np.unique(sensor_list))