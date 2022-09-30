import os

def get_data_config():
	from ultimatelabeling import utils
	
	# conf_dict = utils.load_json('config/annotation_config.json')
	ROOT_DIR = "../bin" #"/media/abdo2020/DATA1/Datasets/toolboxes/UltimateLabeling"
	STATE_PATH = os.path.join(ROOT_DIR, "state.pkl")
	# DATA_DIR = conf_dict['DATA_DIR'] 
	DATA_DIR = '/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/dash-CAM/2022.08.05/VID_015/'
	OUTPUT_DIR = DATA_DIR

	RESOURCES_DIR = os.path.join(ROOT_DIR, "res")
	SERVER_DIR = "UltimateLabeling_server/"
	return STATE_PATH, DATA_DIR, OUTPUT_DIR, RESOURCES_DIR, SERVER_DIR

STATE_PATH, DATA_DIR, OUTPUT_DIR, RESOURCES_DIR, SERVER_DIR = get_data_config()