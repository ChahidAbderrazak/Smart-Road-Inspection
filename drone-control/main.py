
import os
import sys
import time
from lib import sensors_functions
def main():
	for k in range(10):
		# get image
		tic = time.perf_counter()
		frame, err = sensors_functions.get_RGB_Camera()
		toc = time.perf_counter() - tic
		print(f'\n the camera data: \n - frame ={frame}\n - time ={toc} ')
		# get Lidar
		tic = time.perf_counter()
		data, err = sensors_functions.get_Lidar()
		toc = time.perf_counter() - tic
		print(f'\n the Lidar data: \n - data ={data}\n - time ={toc} ')
		# get Data from Drone
		tic = time.perf_counter()
		u=1
		err = sensors_functions.set_drone_control(u)
		data, err = sensors_functions.get_drone_data()
		toc = time.perf_counter() - tic
		print(f'\n the Drone data data: \n - data ={data}\n - time ={toc} ')

		
if __name__ == '__main__':
		main()

