# -*- coding: utf-8 -*-
"""
Created on Wed Sep  7 12:14:10 2022

@author: 100840150
"""

from lib_ahmad.HAIS_project import *

def commpress():
        while True:
            if chek_the_data():
                commpresss(1000000)

def uploading(th=1000000):
        while True:
            if chek_the_data():
                # compress the data
                commpresss(th=th)

            # upload data to Firebase
            if os.listdir(root)!=[]:
                send_data()

                


if __name__ == "__main__":
    commpress()



