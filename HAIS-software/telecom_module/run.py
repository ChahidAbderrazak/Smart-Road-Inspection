# -*- coding: utf-8 -*-
"""
Created on Fri Sep 23 15:10:38 2022

@author: 100840150
"""

from lib.commpression import commpress
from lib.read_data import read
from lib.send_data import send
import threading


def send_data():
  # threads for parallel processing
    t1 = threading.Thread(target=commpress)
    t2 = threading.Thread(target=send)

    t1.start()
    t2.start()
    
def reteive_data():
  # threads for parallel processing
    t1 = threading.Thread(target=read)
    t1.start()
    
    
if __name__ == "__main__":
    send_data()
    #reteive_data()
    
