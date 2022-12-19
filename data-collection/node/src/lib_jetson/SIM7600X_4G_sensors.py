import os
import sys
import time

###### SENSORS packages

try:
	from SIM7600X_4G.GPS import GPS 
except:
	from lib_jetson.SIM7600X_4G.GPS import  GPS

def get_gps_data(visualize=False):
    ''' outputs [lat, lng, alt] '''
    err, dict_GPS = GPS.getGpsPosition()
    if visualize:
        print('\n -GPS data=', dict_GPS)
    return  dict_GPS