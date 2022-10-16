from nuscenes.nuscenes import NuScenes

dataroot='/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/NuScience/nuScenes'
version='v1.0-mini'

# dataroot='/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/hais-node/2022-10-11/UOIT-parking-Abderrazak' 
dataroot='/home/abdo2020/Desktop/demo-hais-data' 
version='v1.0'

# Analyze the Nuscene dataset
nusc_dataloder = NuScenes(version=version, dataroot=dataroot, verbose=True)
################################################################################
print(f'\n\n  1)- List all missions: ')
nusc_dataloder.list_scenes()
################################################################################
print(f'\n\n  2)- Explore a scene: ')
my_scene = nusc_dataloder.scene[0]
print(my_scene)
################################################################################
first_sample_token = my_scene['first_sample_token']
print(f'\n\n  2)- Explore the  sample [{first_sample_token}] of the scene: ')
my_sample = nusc_dataloder.get('sample', first_sample_token)
print(my_sample)
################################################################################