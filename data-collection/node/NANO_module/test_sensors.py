import os, sys
from data_collection.lib.lidar_sensor import *
from data_collection.lib.utils import get_time_tag

#from ..data_collection.lib.lidar_sensor import *

def test_lidar():
	from rplidar import RPLidar
	PORT_NAME='/dev/ttyUSB0'
	print('Stconnecting to {PORT_NAME}')
	lidar = RPLidar(PORT_NAME)

	info = lidar.get_info()
	print(info)

	health = lidar.get_health()
	print(health)

	for i, scan in enumerate(lidar.iter_scans()):
		print('%d: Got %d measurments' % (i, len(scan)))
		if i > 10:
			break

	lidar.stop()
	lidar.stop_motor()
	lidar.disconnect()

def test_vizualize_lidar():
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


def test_camera(disp=False):
	# import the opencv library
	import cv2
	import numpy as np

	size=(1000, 1000)
	# define a video capture object
	camera_id=3
	vid = cv2.VideoCapture(camera_id)
	while True:
		# Capture the video frame
		ret, frame = vid.read()
		if not ret:
			print(f'error: the camera ID {camera_id} is not connected')

		else:
			# frame = cv2.resize(frame, size) 
			cv2.imwrite(f'frame_cam{camera_id}.jpg', frame)
			# Display the resulting frame
			if disp:
				# print(f'\n - image={frame}  \n - pixels={np.unique(frame)}')
				cv2.imshow(f'frame [{frame.shape}]', frame)

	# After the loop release the cap object
	vid.release()
	# Destroy all the windows
	cv2.destroyAllWindows()

# ##--------------------------------  GPS ------------------------------------------
# from gps import *
# import time
# running = True

# def test_gps():
#     def getPositionData(gps):
#         nx = gpsd.next()
#         # For a list of all supported classes and fields refer to:
#         # https://gpsd.gitlab.io/gpsd/gpsd_json.html
#         if nx['class'] == 'TPV':
#             latitude = getattr(nx,'lat', "Unknown")
#             longitude = getattr(nx,'lon', "Unknown")
#             print ("Your position: lon = " + str(longitude) + ", lat = " + str(latitude))

#     gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)

#     try:
#         print ("Application started!")
#         while running:
#             getPositionData(gpsd)
#             time.sleep(1.0)

#     except (KeyboardInterrupt):
#         running = False
#         print ("Applications closed!")


if __name__ == "__main__":
	# test_camera(disp=True)
	test_lidar()
	# test_gps()
    
