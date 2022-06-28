
import os
import sys
from lib import sensors_functions
def main():
	for k in range(10):
		# get image
		frame, err = sensors_functions.get_RGB_Camera()
		print(frame)
		# get sensor2
		data2, err = sensors_functions.get_Lidar()
		print(data2)
		
if __name__ == '__main__':
    main()

