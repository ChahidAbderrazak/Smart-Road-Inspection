import os, sys
try:
	from lib.lidar_sensor import *
	from lib.utils import get_timestamp, get_time_tag
	from lib.config_parameters import *
except:
	from src.lib.lidar_sensor import *
	from src.lib.utils import get_timestamp, get_time_tag
	from src.lib.config_parameters import *


def test_lidar():
	lidar_device = RPLidar_Sensor(PORT_NAME=PORT_NAME, visualize=True)
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
	cam =cam0# cam_left # cam_right# 
	# cam = cv2.VideoCapture(0)
	try:
		# Capture the video frame
		ret, frame = cam.read()
		frame = cv2.resize(frame, size) 
		# Display the resulting frame
		print(f'\n - image size={frame.size}  \n - pixels={np.unique(frame)}')
		cv2.imshow('frame', frame)
		# After the loop release the cap object
		cam.release()
		# Destroy all the windows
		cv2.destroyAllWindows()
	except Exception as e:
		print(f'\n error with the camera sensor! \n Exception: {e}')

if __name__ == "__main__":
	test_camera()
	# test_lidar()
    
