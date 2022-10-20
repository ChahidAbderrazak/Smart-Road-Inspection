import os, sys
from itertools import groupby
from glob import glob
try:
		from lib import utils
except:
		import utils
from datetime import datetime

def get_drone_timestamp(str_time):
	now = datetime.strptime(str_time,'%Y-%m-%d %H:%M:%S.%f')
	now.microsecond
	return datetime.timestamp(now)

def convert_to_time_stamp(str_time):
	now = datetime.strptime(str_time, '%H:%M:%S,%f')
	now.microsecond
	return datetime.timestamp(now)

def sparse_DJI_SRT(filename):
	'''
	Convert SRT file from DJI Movic3 drone into a dict format
	'''
	# "chunk" our input file, delimited by blank lines
	with open(filename) as f:
			res = [list(g) for b,g in groupby(f, lambda x: bool(x.strip())) if b]
	output = []
	lon_list,lat_list, alt_list,metric_list=[],[],[],[]
	for sub in res:
			if len(sub) >= 3: # not strictly necessary, but better safe than sorry
					sub = [x.strip() for x in sub]
					number, start_end, FrameCnt,	DiffTime, content= sub # py3 syntax
					number=int(number)
					Frames=FrameCnt[2:].split(':')[1]
					Frames=int(Frames.split(',')[0])
					start, end = start_end.split(' --> ')
					start=convert_to_time_stamp(start)
					end=convert_to_time_stamp(end)
					DiffTime=get_drone_timestamp(DiffTime)
					content=content.replace('[', '').replace(']', '').replace(':', '')
					location= content.split(' ')
					dict_={"number":number, "start":start, "end":end, "FrameCnt":Frames, "DiffTime":DiffTime}
					for k, line in enumerate(location):
						for metric in ['iso',	'fnum', 'ev','ct',	'focal_len',	'latitude',	'longitude', 'rel_alt', 'abs_alt']:
							if line==metric:
								dict_.update({metric: float(location[k+1])})
					# updaet the location and the metadata
					output.append(dict_)
					lon_list.append(dict_['longitude'])
					lat_list.append(dict_['latitude'])
					alt_list.append(dict_['abs_alt'])
					import random
					road_metric = random.randint(0, 4)
					metric_list.append(road_metric)
	inspection_dict={'lon': lon_list, 'lat':	lat_list, 'alt':	alt_list, 'metric': metric_list}
	return output, inspection_dict

def convert_done_metadata(filename):
	'''
	convert drone metadata to json format
	'''
	output_filename=filename.replace('.SRT', '.json')
	inspection_filename=os.path.join(os.path.dirname(filename), 'inspection_dic.json')
	drone_meta, inspection_dict = sparse_DJI_SRT(filename)
	
	# save the drone metadata
	utils.save_json(drone_meta, output_filename)
	print(f'\n\n The meta data json file saved in : \n {output_filename}')

	# save the drone positions
	utils.save_json(drone_meta, output_filename)
	print(f'\n The inspection json file is saved in : \n {inspection_filename}')
	utils.save_json([inspection_dict], inspection_filename)
	return drone_meta, inspection_dict

def save_mission_json(mission_root, drone_meta,configuration):
	dict_frame_list=[{}]
	for drone_frame in drone_meta:
		# save image file locally
		frame=drone_frame['number']
		image_path_name = "{:05d}.jpg".format(frame)	
		# drone location: drone_location=[lat, lng, alt]
		drone_location=[drone_frame['latitude'],
										drone_frame['longitude'],
										drone_frame['abs_alt']]
		# update outputs
		dict_frame={}
		dict_frame["frame"] = drone_frame['number']
		dict_frame["description"] = configuration["description"]
		dict_frame["timestamp"] = drone_frame['end']
		dict_frame["scene"] = drone_frame['FrameCnt']
		dict_frame["sensor_name"] = "DRONE_CAMERA"
		dict_frame["position"] = {"Translation": drone_location,
															"Rotation": []}
		dict_frame["calibration"] = {"Translation": [], "Rotation": [], "Camera_intrinsic": ""}
		dict_frame["fileformat"] = "jpg"
		dict_frame["filename"] = str(os.path.join("sweeps", "DRONE_CAMERA", image_path_name))
		dict_frame["meta_data"] = drone_frame

		# updaet the frame list 
		dict_frame_list.append(dict_frame)
	# save the mission file
	if len(dict_frame_list)>0:
		date=drone_meta[0]['DiffTime']
		timestamp=datetime.fromtimestamp(date)
		filename_strg=timestamp.strftime("%Y-%m-%d-%Hh-%Mmin-%Ssec") + '.json' 
		mission_path=os.path.join(mission_root, filename_strg)
		utils.save_json(dict_frame_list, mission_path)

def build_Hais_data_strucure(dataroot):
	'''
	Build the predefined data structure od HAIS node
	'''
	list_files= utils.getListOfFiles(dataroot, ext='.SRT')
	print(f'\n - {len(list_files)} file are found : \n {list_files}')

	for filename in list_files:
		# find the videos format
		video_path=''
		for ext in ['.LRF', '.lrf', '.MP4', '.mp4', '.MOV', '.mov']:
			path=filename.replace('.SRT', ext)
			if os.path.exists(path):
				video_path=path
				break
		if video_path=='':
			print(f'\n - Warnning: No videos is found corresponding to drone mission: \n{filename}')
			continue
		# convert the video into image s
		print(f'\n - converting the video: \n{video_path}')
		sweeps_path=os.path.join(os.path.dirname(video_path), "sweeps", "DRONE_CAMERA")
		if not os.path.exists(sweeps_path) or len(glob(os.path.join(sweeps_path,'*')))==0:
			cmd=f"bin/convert_video_to_Images.sh {video_path} {sweeps_path}"
			utils.create_new_directory(sweeps_path)
			os.system(cmd)

		# create the mission info
		configuration={ "vehicle":"drone-M3", "location":"NA",
			"description": "HAIS- Road inspection using Drone-M3"
		}
		info_file_path=os.path.join(os.path.dirname(video_path), "info.json")
		utils.save_json(configuration, info_file_path)
		# convert drone metadata to json format
		drone_meta, inspection_dict=convert_done_metadata(filename)

		# Build the mission json_file:
		mission_root=os.path.join(os.path.dirname(video_path), "missions")
		utils.create_new_directory(mission_root)
		save_mission_json(mission_root, drone_meta,configuration)

def test_drone_data_preparation():
	dataroot='/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/hais-drone/inspection/2022-10-12/UIOT-bridge/bridge2'
	# Build the predefined data structure od HAIS node
	build_Hais_data_strucure(dataroot)

if __name__ == '__main__':
	
	test_drone_data_preparation()
