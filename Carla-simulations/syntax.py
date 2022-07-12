
import numpy as np
from lib import utils
from lib import hais_manager
# filename='/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/Carla_scenes/v0.1/sample_data.json'
# data = utils.load_json(filename)
# format=[]
# for sensor in data:
# 	format.append(sensor['fileformat'])

# print(np.unique(format))
    # input configurations
raw_data = hais_manager.HAIS_bot(config='config.json')
# create the database structure
raw_data.create_database()