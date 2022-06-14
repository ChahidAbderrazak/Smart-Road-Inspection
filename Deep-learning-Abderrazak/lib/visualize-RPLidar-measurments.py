#!/usr/bin/env python3
'''Animates distances and measurment quality'''
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation



class RPLidar_records(object):
		'''Class for communicating with RPLidar rangefinder scanners'''

		def __init__(self, filename, angle_step=1):
				'''Initilize RPLidar object for communicating with the sensor.

				Parameters
				----------
				port : str
						Serial port name to which sensor is connected
				baudrate : int, optional
						Baudrate for serial connection (the default is 115200)
				timeout : float, optional
						Serial port connection timeout in seconds (the default is 1)
				logger : logging.Logger instance, optional
						Logger instance, if none is provided new instance is created
				'''
				self.filename=filename
				self.angle_step=angle_step

		def plot_image(self, time_vec, img):
			plt.imshow(img)#, interpolation='none')
			plt.ylabel('time (sec)')#(time_vec)
			plt.show()

		def plot_lidar(self, num_scans=100):
				scans_dict_list= read_scan(self.filename)
				join_distance_list =[]
				time_vec = []
				for k,	scan in enumerate(scans_dict_list):
					distance_list = scan['lidar/dist_array']
					join_distance_list.append(distance_list)
					time_tag = scan["_timestamp_ms"]/1000
					time_vec.append(time_tag)
					if k==num_scans:
						break
				
				# convert list to array
				length = max(map(len, join_distance_list))
				lidar_img=np.array([xi+[0]*(length-len(xi)) for xi in join_distance_list])
				# print(f'join_distance_list={join_distance_list}')
				print(f'\n\nlidar_img={lidar_img}')
				# plot image
				self.plot_image(time_vec, lidar_img)

		def iter_scans(self, scan_type='normal', max_buf_meas=3000, min_len=5):
				'''Iterate over scans. Note that consumer must be fast enough,
				otherwise data will be accumulated inside buffer and consumer will get
				data with increasing lag.

				Parameters
				----------
				filename : path to where the scans are saved
						
				Yields
				------
				scan : list
						List of the measures. Each measurment is tuple with following
						format: (quality, Azimuth , distance). For values description please
						refer to `iter_measures` method's documentation.
				'''
				scans_dict_list= read_scan(self.filename)
				altitude=0
				quality = -1
				scan_list = []
				for	scan in scans_dict_list:
					distance_list = scan['lidar/dist_array']
					time_tag = scan["_timestamp_ms"]/1000
					Azimuth  = 0
					scan_list = []
					for distance in distance_list :
						Azimuth  = Azimuth  + self.angle_step
						if distance > 0:
							scan_list.append((quality, Azimuth , distance))
					print(f'\t\t - time= {time_tag} sec , total Azimuth ={Azimuth} deg, distance={distance}, Azimuith={altitude}')
					yield scan_list

def update_line(num, iterator, line):
	print(f' --> new Scan ID={num}')
	scan = next(iterator)
	offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
	line.set_offsets(offsets)
	intens = np.array([meas[0] for meas in scan])
	line.set_array(intens)
	return line,

def read_scan(filename):
	import json
	# Using readlines()
	file1 = open(filename, 'r')
	Lines = file1.readlines()
	count = 0
	scans_dict_list=[]
	# Strips the newline character
	for line in Lines:
			count += 1
			# print("Line{}: {}".format(count, line.strip()))
			scans_dict_list.append( json.loads(line.strip()) )
	return scans_dict_list

def run_records():
	DMAX = 100
	IMIN = 0
	IMAX = 50
	filename='/media/abdo2020/DATA1/Datasets/numerical-dataset/RPLidar-data/catalog_0_A1_R5_465_1.catalog.txt'# catalog_0_A2_465_2.catalog.txt'#catalog_0_A1_R6_465_1.catalog.txt'#
	lidar = RPLidar_records(filename, angle_step=2)
	# vizualise lidar image
	lidar.plot_lidar()

	# lidat animation
	fig = plt.figure()
	ax = plt.subplot(111, projection='polar')
	line = ax.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX],
													cmap=plt.cm.Greys_r, lw=0)
	ax.set_rmax(DMAX)
	ax.grid(True)
	
	iterator = lidar.iter_scans()  
	ani = animation.FuncAnimation(fig, update_line,
				fargs=(iterator, line), interval=50)
	plt.show()

if __name__ == '__main__':
		run_records()
