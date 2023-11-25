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
	Convert SRT file from DJI Mavic3 drone into a dict format
	'''
	# "chunk" our input file, delimited by blank lines
	with open(filename) as f:
			res = [list(g) for b,g in groupby(f, lambda x: bool(x.strip())) if b]
	output = []
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
					# update the location and the metadata
					output.append(dict_)
					
	return output

def convert_done_metadata(filename):
	'''
	convert drone metadata to json format
	'''
	output_filename=filename.replace('.SRT', '.json')
	drone_meta = sparse_DJI_SRT(filename)
	
	# save the drone metadata
	utils.save_json(drone_meta, output_filename)
	print(f'\n\n The meta data json file saved in : \n {output_filename}')

	# save the drone positions
	utils.save_json(drone_meta, output_filename)
	return drone_meta

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

		# update the frame list 
		dict_frame_list.append(dict_frame)
	# save the mission file
	if len(dict_frame_list)>0:
		date=drone_meta[0]['DiffTime']
		timestamp=datetime.fromtimestamp(date)
		filename_strg=timestamp.strftime("%Y-%m-%d-%Hh-%Mmin-%Ssec") + '.json' 
		mission_path=os.path.join(mission_root, filename_strg)
		utils.save_json(dict_frame_list, mission_path)

def build_Hais_data_structure(dataroot, ext='.SRT'):
	'''
	Build the predefined data structure of HAIS node
	'''
	# display
	print(f'\n- Converting the drone data to HAIS database structure. Please wait ...')
	# check the root:
	if os.path.isfile(dataroot):
		dataroot=os.path.dirname(dataroot)
	# find the drone video
	list_files= utils.getListOfFiles(dataroot, ext=ext, path_pattern='', allFiles=[])
	print(f'\n - {len(list_files)} drone videos are found : \n {list_files}')

	for filename in list_files:
		# find the videos format
		video_path=''
		# extension = os.path.splitext(filename)[1][1:]
		for ext_ in ['.LRF', '.lrf', '.MP4', '.mp4', '.MOV', '.mov']:
			path=filename.replace('.SRT', ext_)
			if os.path.exists(path):
				video_path=path
				break
		if video_path=='':
			print(f'\n - Warning: No videos is found corresponding to drone mission: \n{filename}')
			continue
		# convert the video into images
		print(f'\n - converting the video: \n  {filename} \n{video_path}')
		sweeps_path=os.path.join(os.path.dirname(video_path), "sweeps", "DRONE_CAMERA")
		nb_images=len(glob(os.path.join(sweeps_path,'*')))
		if not os.path.exists(sweeps_path) or nb_images==0:
			cmd=f"lib/convert_video_to_Images.sh {video_path} {sweeps_path}"
			utils.create_new_directory(sweeps_path)
			os.system(cmd)
		else:
			print(f'\n\t- the video file [ {video_path} ] is already converted to {nb_images} images')

		# create the mission info
		config_file=os.path.join(dataroot, 'info.json')

		configuration=utils.load_json(config_file)
		if configuration==[]:
			configuration={ "vehicle":"drone-TDB", 
											"location":"NA",
											"description": "HAIS- Road inspection using a Drone"
										}
			# sve the data info
			info_file_path=os.path.join(os.path.dirname(video_path), "info.json")
			utils.save_json(configuration, info_file_path)
		# convert drone metadata to json format
		drone_meta=convert_done_metadata(filename)

		# Build the mission json_file:
		mission_root=os.path.join(os.path.dirname(video_path), "missions")
		utils.create_new_directory(mission_root)
		save_mission_json(mission_root, drone_meta,configuration)

################################################################################
def test_drone_data_preparation():
	dataroot='../data/download/drone1'
	# Build the predefined data structure od HAIS node
	build_Hais_data_structure(dataroot)

if __name__ == '__main__':
	
	test_drone_data_preparation()
