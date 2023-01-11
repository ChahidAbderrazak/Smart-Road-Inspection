import os
import sys
# sys.path.insert(1, '~/catkin_ws/src/hais/src/src/lib')
# sys.path.insert(2, '~/catkin_ws/src/hais/src/src/lib_RPI')
# sys.path.insert(3, '~/catkin_ws/src/hais/src/src/lib_jetson')
try:
	from lib.utils import get_timestamp, get_time_tag, create_new_folder
	from lib.config_parameters import *
except:
	from src.lib.utils import get_timestamp, get_time_tag, create_new_folder
	from src.lib.config_parameters import *
# RPI
# from lib_RPI import lidar_sensor

# Jetson Nano
from lib_jetson import ENV_IMU_sensors, SIM7600X_4G_sensors

def test_lidar():
	print('\n --> testing the RPLIDAR sensor')
	lidar_device = lidar_sensor.RPLidar_Sensor(PORT_NAME=PORT_NAME, visualize=True)
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
	print('\n --> testing the camera sensor')
	import cv2
	import numpy as np
	for  cam_id in range(10):
		try:
			# define a video capture object
			vid = cv2.VideoCapture(cam_id)
			# Capture the video frame
			# by frame
			ret, frame = vid.read()
			if frame==None:
				print('\n No Camera is conneted to port ['+str(cam_id)+']') 
				continue

			print('\n - image size= ', frame.shape)
			#frame = cv2.resize(frame, size) 
			filename='files/cam_' + str(cam_id) + '.jpg'
			create_new_folder(os.path.dirname(filename))
			cv2.imwrite(filename, frame)
			# Display the resulting frame
			cv2.imshow('frame', frame)
		except Exception as e:
			print('\n error with the camera port ['+str(cam_id)+']') ;# print(' Exception:', e)
	# After the loop release the cap object
	vid.release()
	# Destroy all the windows
	cv2.destroyAllWindows()

def run_RPi_testing():
	print('\n\n --- Testing RPi Nano sensors')
	test_camera()
	# test_lidar()

def run_Jetson_testing():
	print('\n\n --- Testing Jetson Nano sensors')
	test_camera()
	# print('\n --> testing the IMU sensor')
	# dict_Kinetic=ENV_IMU_sensors.get_IMU_data(LCD_visualize=False)

	# print('\n --> testing the ENV sensor')
	# dict_wheather=ENV_IMU_sensors.get_wheather_sensors_data(LCD_visualize=False)

	# print('\n --> testing the GPS sensor')
	# dict_GPS = SIM7600X_4G_sensors.get_gps_data(visualize=True)


if __name__ == "__main__":
	# run_RPi_testing()
	run_Jetson_testing()
    
