#!/usr/bin/env python3
'''Animates distances and measurment quality'''
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

class RPLidar(object):
	'''Class for communicating with RPLidar rangefinder scanners : sampling time = 5seconds'''

	def __init__(self, filename, angle_step=1,DMAX=100, IMIN=0, IMAX=50):
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
		self.DMAX=DMAX
		self.IMIN=IMIN
		self.IMAX=IMAX

	def plot_image(self, time_vec, img):
		fig = plt.figure(figsize=(8,15))
		plt.rc('font', size=18) 
		plt.imshow(img.T)#, interpolation='none')
		plt.xlabel('time (sec)') 
		plt.ylabel('distance per angle')
		plt.colorbar()
		plt.show()

	def plot_lidar(self, num_scans=100):
		scans_dict_list= self.read_scan(self.filename)
		join_distance_list =[]
		time_vec = []
		for k,	scan in enumerate(scans_dict_list):
			distance_list = scan['lidar/dist_array']
			join_distance_list.append(distance_list)
			time_tag = scan["_timestamp_ms"]/1000
			time_vec.append(time_tag)
			if k%100==0:
				print(f'\n\n distance_list [{len(distance_list)} deg]={distance_list}')
			if k==num_scans:
				break
		
		# convert list to array
		length = max(map(len, join_distance_list))
		lidar_img=np.array([xi+[0]*(length-len(xi)) for xi in join_distance_list])
		# print(f'join_distance_list={join_distance_list}')
		# print(f'\n\nlidar_img [size: {lidar_img.shape}]={lidar_img}')

		# plot image
		self.plot_image(time_vec, lidar_img)

	def plot_lidar_animation(self):
		# lidar animation
		fig = plt.figure(figsize=(15,15))
		plt.rc('font', size=18) 
		ax = plt.subplot(111, projection='polar')
		line = ax.scatter([0, 0], [0, 0], s=10, c=[self.IMIN, self.IMAX],
														cmap=plt.cm.Greys_r, lw=1)
		ax.set_rmax(self.DMAX)
		ax.grid(True)
		
		iterator = self.iter_scans()  
		ani = animation.FuncAnimation(fig, self.update_line,
				fargs=(iterator, line), interval=50)
		plt.show()

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
		scans_dict_list= self.read_scan(self.filename)
		altitude=0
		quality = -1
		scan_list = []
		for k, scan in enumerate(scans_dict_list):
			distance_list = scan['lidar/dist_array']
			time_tag = scan["_timestamp_ms"]/1000
			Azimuth  = 0
			scan_list = []
			for distance in distance_list :
				Azimuth  = Azimuth  + self.angle_step
				if distance > 0:
					scan_list.append((quality, Azimuth , distance))
			if k%10==0:
				print(f'\n--> new Scan ID:{k} \t time= {time_tag} sec , total Azimuth ={Azimuth} deg, distance={distance}, Azimuith={altitude}')
			yield scan_list

	def update_line(self, num, iterator, line):
		scan = next(iterator)
		offsets = np.array([(np.radians(meas[1]), meas[2]) for meas in scan])
		line.set_offsets(offsets)
		intens = np.array([meas[0] for meas in scan])
		line.set_array(intens)
		return line,

	def read_scan(self, filename):
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

def run_Lidar_records():
	DMAX = 100
	IMIN = 0
	IMAX = 50
	filename='/media/abdo2020/DATA1/Datasets/numerical-dataset/RPLidar-data/catalog_0_A1_R5_465_1.catalog.txt'# catalog_0_A2_465_2.catalog.txt'#catalog_0_A1_R6_465_1.catalog.txt'#
	lidar = RPLidar(filename, angle_step=2, DMAX=DMAX, IMIN=IMIN, IMAX=IMAX)
	# vizualise lidar image
	lidar.plot_lidar()
	lidar.plot_lidar_animation()

if __name__ == '__main__':
		run_Lidar_records()
