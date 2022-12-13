# -*- coding: utf-8 -*-
"""
Created on Tue Sep  6 12:57:46 2022

@author: 100840150
"""

import pandas as pd 
import matplotlib.pyplot as plt
data1=pd.read_csv(r'C:\Users\100840150\Desktop\HAIS_DEMO\result\file with size 1, 241, 088.csv')
data2=pd.read_csv(r'C:\Users\100840150\Desktop\HAIS_DEMO\result\file with size 618, 496.csv')
data3=pd.read_csv(r'C:\Users\100840150\Desktop\HAIS_DEMO\result\file with size 245, 760.csv')
data4=pd.read_csv(r'C:\Users\100840150\Desktop\HAIS_DEMO\result\file with size 151, 552.csv')
data5=pd.read_csv(r'C:\Users\100840150\Desktop\HAIS_DEMO\result\file with size 81, 920.csv')

x=[i for i in range (15522, 1407872, 5000)]
y1=data5.iloc[:, 1].values
y2=data4.iloc[:, 1].values
y3=data3.iloc[:, 1].values
y4=data2.iloc[:, 1].values
y5=data1.iloc[:, 1].values

fig = plt.figure()

plt.errorbar(x, y1, xerr=0.1, label='size 81, 920')

plt.errorbar(x , y2, xerr=0.1, label='size 151, 552')

plt.errorbar(x , y3, xerr=0.1, label='size 245, 760')
plt.errorbar(x , y4, xerr=0.1, label='size 618, 496')

plt.errorbar(x , y5, xerr=0.1, label='size 1, 241, 088')

plt.legend()
plt.show()