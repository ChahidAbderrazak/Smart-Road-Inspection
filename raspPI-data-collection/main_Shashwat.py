import os
import sys
import time
from lib import utils

config = {"Scenario": "S1", "USE_CASE": "UC1", "duration": 10,
          "vehicle": "car1", "location": "Ontario Tech University",
          "description": "low-safety index"}

data_root = "/home/manir/hais/Shashwat/Hais_Data"  # storage path on local device

if __name__ == "__main__":

    # create json file
    filename_strg, dir_storage = utils.get_file_names(configuration)
    filename = os.path.join(data_root, "sweeps", "Json_files", filename_strg)
    utils.save_json([{}], filename)

    # set up folder to save data locally
    utils.create_databse_folders()

    # initialise pygame
    pygame.init()
    print("pygame initiated")

    # threads for parallel processing
    t1 = threading.Thread(target=utils.save_json_file)
    t2 = threading.Thread(target=utils.save_image_data)
    t3 = threading.Thread(target=utils.save_accelerometer_data)

    t1.start()
    t2.start()
    t3.start()

    pygame.quit()
