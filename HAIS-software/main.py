
from lib import hais_database

# from telecom_module.lib import utils_telecom
# def sync_data_to_firebase():
#     #  retreive the the inspection data from the cloud
#     utils_telecom.download_data()

#     # send the inspection data to the cloud
#     utils_telecom.upload_data()

def convert_measurment_to_database():
	dataroot="/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/hais-node/2022-10-11/UOIT-parking-Abderrazak"
	dataroot='/home/abdo2020/Desktop/demo-hais-data' 
	version='v1.0'
	# Loaf the collected inspection sensors dataset
	raw_data = hais_database.HAIS_node(dataroot=dataroot, version=version)

	# create the database structure
	raw_data.create_database()

	# Show a scene 
	scene_=raw_data.scene
	print(f'\n\n - scene= {scene_}')

	# # annotate the a scene
	# raw_data.annotate_database(scene=scene_[0])

if __name__ == "__main__":

    # convert the sensors measurment to a structured HAIS-database: "/media/abdo2020/DATA1/Datasets/data-demo/hais-inspection/",
    convert_measurment_to_database()

    # # synchorize (upload/download) sensors measurement
    # sync_data_to_firebase()




