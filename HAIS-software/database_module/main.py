# # create  conda : https://github.com/nutonomy/nuscenes-devkit/blob/master/docs/installation.md
# pip install nuscenes-devkit
import glob
import os
import sys
import time
import random

# Load the Nscene datset 
# from nuimages import NuImages
from nuscenes import NuScenes
dataroot='/run/user/1001/gvfs/smb-share:server=sesl-cloud.local,share=hais-project/dataset/nuScenes'
dataroot='/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/NuScience/nuScenes'

nusc = NuScenes(version='v1.0-mini', dataroot=dataroot, verbose=True)

#
my_scene = nusc.scene[0]
first_sample_token = my_scene['first_sample_token']
my_sample = nusc.get('sample', first_sample_token)


# # Show the annotations + add the inspection lables : safety index
# sensor = 'CAM_FRONT'
# cam_front_data = nusc.get('sample_data', my_sample['data'][sensor])
# nusc.render_sample_data(cam_front_data['token'])

# show lidar + image annatation
my_annotation_token = "70aecbe9b64f4722ab3c230391a3beb8" #my_sample['anns'][18]
my_annotation_metadata =  nusc.get('sample_annotation', my_annotation_token)
nusc.render_annotation(my_annotation_token)
print(f'\n end')

#  show the map
nusc.render_egoposes_on_map(log_location='singapore-onenorth')



# # show the prediction Nuscenes-Map
# from lib.nuscenes.prediction.input_representation.static_layers import StaticLayerRasterizer
# from lib.nuscenes.prediction.input_representation.agents import AgentBoxesWithFadedHistory
# from lib.nuscenes.prediction.input_representation.interface import InputRepresentation
# from lib.nuscenes.prediction.input_representation.combinators import Rasterizer



# use the Geo-datframe to display the safety index on the  Nuscenes-Map 

