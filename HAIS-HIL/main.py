
import os, sys
from lib import hais_database, dji_drone, utils

def convert_measurment_to_database(dataroot):
	'''
	Build a Nuscenes-like data base using hais-node and hais-drone data
	'''
 	# If Drone, build the predefined data structure od HAIS node
	dji_drone.build_Hais_data_strucure(dataroot)


def create_HAIS_database(dataroot, version='v0.0'):
	# Load the collected inspection sensors dataset
	raw_data = hais_database.HAIS_database(dataroot=dataroot, version=version)

	# create the database structure
	raw_data.create_database()

	# # annotate the a scene
	# raw_data.annotate_database(scene=scene_[0])

def  explore_database(dataroot, version):
	'''
	Explore the created database
	'''
	from nuscenes.nuscenes import NuScenes
	nusc = NuScenes(version=version, dataroot=dataroot, verbose=True)

	# Show a scene 
	my_scene=nusc.scene[0]
	print(f'\n\n ==> List of recorded scenes: \n ')
	nusc.list_scenes()

	print(f'\n\n ==> First scene: \n {my_scene}')
	
	my_sample = nusc.get('sample', my_scene['first_sample_token'])
	print(f'\n\n ==> First sample of the scene: \n {my_sample}')
	# nusc.list_sample(my_sample['token'])
	
	# sensor_list =  list(my_sample['data'].keys())
	# print(f'\n\n ==> List of sensors: \n {sensor_list}')
	# sensor=sensor_list[0]
	# cam_front_data = nusc.get('sample_data', my_sample['data'][sensor])
	# try:
	# 	print(f'\n\n ==> First sensor data:  {sensor}: \n {cam_front_data}')
	# 	nusc.render_sample_data(cam_front_data['token'])
	# except Exception as e:
	# 	print(f'\n - Error: cannot render the data of the sensor: {sensor} \
	# 		      \n - Exception: {e}')


if __name__ == "__main__":

	version='v1.0'
	root='/media/abdo2020/DATA1/data/raw-dataset/hais-node'
	# root='/media/abdo2020/DATA1/data/raw-dataset/hais-drone'
	list_datasets=utils.getListOfFiles(root, ext='json', path_pattern='info.json') 
	dataroot_list=[ os.path.dirname(path) for path in list_datasets if 'medium-speed' not in path]
	dataroot_list.sort()
	input(f' --> list of {len(dataroot_list)} available collected data : \n{dataroot_list}')
	# dataroot_list=['/media/abdo2020/DATA1/data/raw-dataset/hais-node/2022-10-11/UOIT-parking-Abderrazak']
	for dataroot in dataroot_list:
		# Create the  HAIS database
		create_HAIS_database(dataroot=dataroot, version=version)

		# Explore the created database
		explore_database(dataroot, version)

		# # synchorize (upload/download) sensors measurement
		# sync_data_to_firebase()





