# -*- coding: utf-8 -*-
"""
Created on Tue Sep  6 11:15:54 2022

@author: 100840150
"""
import os
from HAIS_project import *
import shutil
import pandas as pd

import matplotlib.pyplot as plt
import numpy as np





src=r'C:\Users\100840150\Desktop\HAIS_DEMO\datareceiver - Copy'
dest=r'C:\Users\100840150\Desktop\HAIS_DEMO\datareceiver'
def copy():
     lis=os.listdir(src)
     for ele in lis:
         shutil.copy(os.path.join(src,ele),os.path.join(dest,ele))

def main(start=15522,end=1407872,step=90000):
    x=[]
    y=[]
    for i in range(start,end,step):
        copy()
        x.append(i)
        start_time = time.time()
        commpress(i)
        send_data()
        y.append(time.time() - start_time)
    
    name=os.path.getsize(src)
    dec={'time':x,'th':y}
    dec=pd.DataFrame(dec)
    dec.to_csv(f'file with size {name}.csv',index=False)
        
        
    fig, ax = plt.subplots()
    ax.plot(x, y)
    
    ax.set(xlabel='th', ylabel='time ',
           title=f'file with size {name}')
    ax.grid()
    
    fig.savefig(f"{name}.png")
    plt.show()
        
    
    
    

    
    
if __name__ == "__main__":
    main()
    