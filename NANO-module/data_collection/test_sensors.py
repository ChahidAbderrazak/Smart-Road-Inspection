import os, sys
from lib.lidar_sensor import *
from data_collection.lib.utils import get_timestamp, get_time_tag

def test_lidar():
	lidar_device = RPLidar_Sensor(PORT_NAME='/dev/ttyUSB0',visualize=True)
	
	scan_data = [0]*360
	try:
			
			for scan in lidar_device.lidar.iter_scans():
					for (_, angle, distance) in scan:
							scan_data[min([359, floor(angle)])] = distance
					lidar_d=lidar_device.process_lidar_data(scan_data)
					lidar_device.obj_coord_list=lidar_d
					print(str(get_time_tag(type=1)))

	except KeyboardInterrupt:
			print('Stoping.')
			lidar_device.lidar.stop()
			lidar_device.lidar.stop_motor()
			lidar_device.lidar.disconnect()
			print('Lidar stopped!!.')

def test_camera():
	# import the opencv library
	import cv2
	import numpy as np

	size=(1000, 1000)
	# define a video capture object
	vid = cv2.VideoCapture(0)

	# Capture the video frame
	# by frame
	ret, frame = vid.read()
	frame = cv2.resize(frame, size) 

	# Display the resulting frame
	print(f'\n - image={frame}  \n - pixels={np.unique(frame)}')
	cv2.imshow('frame', frame)
	# After the loop release the cap object
	vid.release()
	# Destroy all the windows
	cv2.destroyAllWindows()

if __name__ == "__main__":
	# test_camera()
	test_lidar()
    
