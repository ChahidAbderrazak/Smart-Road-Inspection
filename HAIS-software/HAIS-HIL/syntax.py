import os, tqdm, sys
from glob import glob
import numpy as np
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

def convert_JetsonNano_database(root, disp=False):
	global 	time_idx, data_dict, scene_sensor, scene_sensor, frame, scene, \
					car_location, timestamp_line, time_idx, configuration
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
