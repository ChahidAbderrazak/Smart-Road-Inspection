# from nuscenes.nuscenes import NuScenes

# dataroot='/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/NuScience/nuScenes'
# version='v1.0-mini'

# # dataroot='/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/hais-node/2022-10-11/UOIT-parking-Abderrazak' 
# dataroot='/home/abdo2020/Desktop/demo-hais-data' 
# version='v1.0'

# # Analyze the Nuscene dataset
# nusc_dataloder = NuScenes(version=version, dataroot=dataroot, verbose=True)
# ################################################################################
# print(f'\n\n	1)- List all missions: ')
# nusc_dataloder.list_scenes()
# ################################################################################
# print(f'\n\n	2)- Explore a scene: ')
# my_scene = nusc_dataloder.scene[0]
# print(my_scene)
# ################################################################################
# first_sample_token = my_scene['first_sample_token']
# print(f'\n\n	2)- Explore the	sample [{first_sample_token}] of the scene: ')
# my_sample = nusc_dataloder.get('sample', first_sample_token)
# print(my_sample)
# ################################################################################
from datetime import datetime


# def get_drone_timestamp(str_time):
# 	date, time_ = str_time.split(' ')
# 	year, month, day = date.split('-')
# 	time_, ms = time_.split('.')
# 	hour, min, sec= time_.split(':')
# 	a=datetime(int(year), int(month), int(day), int(hour), int(min), int(sec))
# 	a.microsecond
# 	return datetime.timestamp(a)

def get_drone_timestamp(str_time):
	now = datetime.strptime(str_time,'%Y-%m-%d %H:%M:%S.%f')
	now.microsecond
	return datetime.timestamp(now)

str_time="2022-10-12 14:09:41.488"
timestamp=get_drone_timestamp(str_time)
print(f' time={str_time} timestamp={timestamp}')

date = datetime.fromtimestamp(timestamp)
print(f' date={date} ')
