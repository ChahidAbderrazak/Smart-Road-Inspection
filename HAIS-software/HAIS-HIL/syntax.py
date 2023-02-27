import os, tqdm, sys
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

def timestamp_from_fileanme(sensor_name, curr_sample):
	global data_dict
	idx=curr_sample[sensor_name][0]
	filename_path=data_dict[sensor_name][idx][2]
	timestamp=get_timestamp(os.path.basename(filename_path) )
	return timestamp

def fill_sample_new_data(sensor_name, curr_sample, frame, scene):
	global data_dict, scene_sensor, car_location, configuration
	# get sensors data
	idx=curr_sample[sensor_name][0]
	filename_path=data_dict[sensor_name][idx][2]
	timestamp=get_timestamp(os.path.basename(filename_path) )
	# print(f'\n --> sensor_name={sensor_name} \n {filename}')
	# print(f'\n - saving data_ sample ...')
	description=""
	filename=os.path.join('sweeps', filename_path.split('sweeps')[1][1:])
	# input(f'\n flag: filename={filename}')
	if "3D" in sensor_name:
		filename_path=glob(filename_path[:-4] + '_*')[0]
		meta_data=os.path.join('sweeps', filename_path.split('sweeps')[1][1:])
	else:
		meta_data=""

	# push new data pipeline
	if curr_sample[sensor_name][0]<curr_sample[sensor_name][1]-1:
		curr_sample[sensor_name][0]+=1

	# # check the if it a new scene
	# if sensor_name in scene_sensor:
	# 	scene_sensor=[]
	# 	scene+=1
	# else:
	# 	scene_sensor.append(sensor_name)
	
	# build the sample data 
	sensor_dict = get_new_sensor_dict(sensor_name, 
																		filename=filename,
																		sensor_frame=frame, 
																		sensor_scene=scene, 
																		car_location=car_location, 
																		timestamp=timestamp, 
																		description=description, 
																		meta_data=meta_data)
	return  sensor_dict


# if float(timestamp)>float(scene_time_stamp): # sensor file is not within the scene  time stamp
# 		# print(f'\n - warrning: sensor timestamp={timestamp} is not within the scene timestamp={scene_time_stamp}')
# 		return {}
# 	else:
	

def convert_JetsonNano_database(root):
	global time_idx, data_dict, scene_sensor, scene_sensor, frame, scene, car_location, timestamp, configuration
	scene_sensor=[]
	scene=-1
	time_idx=0
	car_location=[-1, -1,-1]
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

		print(f'\n - {sensor_name} has {len(list_files)} file')
		data_dict.update({sensor_name:list_files})
		if len(list_files)>0:
			curr_sample.update({sensor_name:[0,len(list_files)]})

		if len(list_files)>N:
			N=len(list_files)
			ref_sensor_name=sensor_name
		
	frame=0
	for mission_json_path in tracker_json:
		mission_dict=utils.load_json(mission_json_path)

		# update the mission json file
		print(f'\n update the mission json file. \
						\n - timestamp sensor= {ref_sensor_name}with {N} files \
						\n - GPS/IMU json have {len(mission_dict)} timestamps ')
		for k in enumerate(range(N)):
			scene+=1
			timestamp=timestamp_from_fileanme(ref_sensor_name, curr_sample)
			try:
				data_sample=mission_dict[time_idx]
				skip_IMU_GPS=False
			except:
				skip_IMU_GPS=True
			if skip_IMU_GPS and data_sample!={}:
				# get the mission sensor in Json file
				sensor_name=data_sample["sensor_name"]
				car_location=data_sample["position"]["Translation"]
				scene_time_stamp=data_sample["timestamp"]
				print(f'\n flag: data_sample0={data_sample}')
				# update the mission dict
				data_sample["frame"]=frame
				data_sample["scene"]=scene
				# input(f'\n flag: data_sample1={data_sample}')
				mission_data_list.append(data_sample)
				frame+=1
				time_idx+=1

			for sensor_name in curr_sample.keys(): # sensor with data files
				# get new sensors sample_data
				new_data= fill_sample_new_data(sensor_name=sensor_name, 
																			curr_sample=curr_sample,
																			frame=frame, 
																			scene=scene)
				# update the mission dict
				if new_data!={}:
					mission_data_list.append(new_data)
					frame+=1

		# display 
		print(f'\n ==> mission file create [ {len(mission_data_list)} data samples')
		mission_json=mission_json_path.replace('log', 'missions')
		utils.create_new_directory(os.path.dirname(mission_json))
		utils.save_json(mission_data_list, mission_json)
		


# root='/media/abdo2020/DATA1/data/raw-dataset/hais-node/2023-02-17/CS1_Oshawa-road'
root='/home/abdo2020/Downloads/Jetson_ROS_data/Node2/mission_2023-02-26-15h-49min-07sec'
# path=get_sensor_filename(sensor_name='fg', frame=0,time_tag='', tag='')
# tsmp1=get_timestamp(path)

# path=get_sensor_filename(sensor_name='rt', frame=0,time_tag='', tag='')
# tsmp2=get_timestamp(path)

# print(f'tsmp1={tsmp1}, tsmp2={tsmp2}')
# print(f'comp={tsmp1>tsmp2}')

# path1='/media/abdo2020/DATA1/data/raw-dataset/hais-node/2023-02-17/CS1_Oshawa-road/sweeps/RIGHT_CAMERA/2023-02-17-17h-30min-51sec__RIGHT_CAMERA__2660.jpg'
# path2='/media/abdo2020/DATA1/data/raw-dataset/hais-node/2023-02-17/CS1_Oshawa-road/sweeps/RIGHT_CAMERA/2023-02-17-17h-30min-57sec__RIGHT_CAMERA__2666.jpg'
# tsmp2=get_timestamp(path2)
# tsmp1=get_timestamp(path1)
# print(f'tsmp1={tsmp1}, tsmp2={tsmp2}')
# print(f'comp={tsmp1<tsmp2}')


convert_JetsonNano_database(root)