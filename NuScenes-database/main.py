# # create  conda : https://github.com/nutonomy/nuscenes-devkit/blob/master/docs/installation.md
# pip install nuscenes-devkit
import glob
import os
import sys
import time
import random

from lib import carla_utils


# Load the Nscene datset 
from lib.nuimages import NuImages
dataroot='/run/user/1001/gvfs/smb-share:server=sesl-cloud.local,share=hais-project/dataset/nuScenes'
nuim = NuImages(dataroot=dataroot, version='v1.0-mini', verbose=True, lazy=True)


# Show the annotations + add the inspection lables : safety index


# show the prediction Nuscenes-Map
from lib.nuscenes.prediction.input_representation.static_layers import StaticLayerRasterizer
from lib.nuscenes.prediction.input_representation.agents import AgentBoxesWithFadedHistory
from lib.nuscenes.prediction.input_representation.interface import InputRepresentation
from lib.nuscenes.prediction.input_representation.combinators import Rasterizer



# use the Geo-datframe to display the safety index on the  Nuscenes-Map 

