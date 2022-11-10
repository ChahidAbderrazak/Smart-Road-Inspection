# -*- coding: utf-8 -*-
"""
Created on Fri Sep 23 15:10:38 2022

@author: 100840150
"""

from lib.commpression import commpress
from lib.send_data import send
import threading



if __name__ == "__main__":
    # threads for parallel processing
    t1 = threading.Thread(target=commpress)
    t2 = threading.Thread(target=send)

    t1.start()
    t2.start()

    #pygame.quit()