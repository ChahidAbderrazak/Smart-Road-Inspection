

from telecom_module.lib import utils_telecom


if __name__ == "__main__":

    # retreive the the inspection data from the cloud
    utils_telecom.download_data()

    # send the inspection data to the cloud
    utils_telecom.upload_data()




