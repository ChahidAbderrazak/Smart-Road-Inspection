import os

def get_data_config():
	from lib import utils
	
	conf_dict = utils.load_json('config/annotation_config.json')
	ROOT_DIR = "/media/abdo2020/DATA1/Datasets/toolboxes/UltimateLabeling"
	
	STATE_PATH = os.path.join(ROOT_DIR, "state2.pkl")
	DATA_DIR = conf_dict['DATA_DIR'] 
	OUTPUT_DIR = DATA_DIR
	RESOURCES_DIR = os.path.join(ROOT_DIR, "res")
	SERVER_DIR = "UltimateLabeling_server/"
	return STATE_PATH, DATA_DIR, OUTPUT_DIR, RESOURCES_DIR, SERVER_DIR

STATE_PATH, DATA_DIR, OUTPUT_DIR, RESOURCES_DIR, SERVER_DIR = get_data_config()