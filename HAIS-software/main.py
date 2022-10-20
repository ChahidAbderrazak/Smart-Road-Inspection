
from lib import hais_database, dji_drone, utils

def convert_measurment_to_database(dataroot, version):
	'''
	Build a Nuscenes-like data base using hais-node and hais-drone data
	'''
 	# If Drone, build the predefined data structure od HAIS node
	dji_drone.build_Hais_data_strucure(dataroot)

	# Load the collected inspection sensors dataset
	raw_data = hais_database.HAIS_node(dataroot=dataroot, version=version)

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
	
	sensor_list =  list(my_sample['data'].keys())
	print(f'\n\n ==> List of sensors: \n {sensor_list}')
	sensor=sensor_list[0]
	cam_front_data = nusc.get('sample_data', my_sample['data'][sensor])
	print(f'\n\n ==> First sensor data:  {sensor}: \n {cam_front_data}')
	nusc.render_sample_data(cam_front_data['token'])

if __name__ == "__main__":
	dataroot="/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/hais-node/2022-10-11/UOIT-parking-Abderrazak"
	dataroot='/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/hais-drone/inspection/2022-10-12/UIOT-bridge/bridge'
	# dataroot='/media/abdo2020/DATA1/Datasets/data-demo/demo-hais-data'
	version='v1.0'

	# # convert the sensors measurment to a structured HAIS-database
	# convert_measurment_to_database(dataroot, version)

	# Explore the created database
	explore_database(dataroot, version)

	# # synchorize (upload/download) sensors measurement
	# sync_data_to_firebase()





