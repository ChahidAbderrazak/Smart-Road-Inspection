# -*- coding: utf-8 -*-
"""
Created on Sun Jul 24 10:12:26 2022

@author: 100840150
"""

from lib.HAIS_project import *
def read():
    while True:
        err=get_data()
        if not err:
            print(f'\n- Decompressing the downloaded data ...')  
            decommpress()
if __name__ == "__main__":
    read()
    
