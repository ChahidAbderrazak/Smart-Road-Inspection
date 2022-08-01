# -*- coding: utf-8 -*-
"""
Created on Sun Jul 24 10:04:01 2022

@author: 100840150
"""

from lib.utils_telecom import *

def download_data():
    get_data()
    

def upload_data():
        while True:
            if chek_the_data():
                rename()
                send_data()
                time.sleep(0.5)
            else:
                print('there is no data yet')
                time.sleep(0.5)

if __name__ == "__main__":
    # send the inspection data to the cloud
    upload_data()

    #  retreive the the inspection data from the cloud
    download_data()



