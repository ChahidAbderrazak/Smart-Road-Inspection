#!/usr/bin/env python3
'''Animates distances and measurement quality'''
from cProfile import label
import os, time
import matplotlib.pyplot as plt
import numpy as np
from glob import glob
import matplotlib.animation as animation
from tqdm import tqdm

try:
    from lib import utils
except:
    import utils

colors_list=['#0051a2', '#97964a', '#ffd44f', '#f4777f', '#93003a',"#E69F00", "#56B4E9", "#009E73", "#0072B2", "#D55E00", "#CC79A7", "#F0E442"]

########## FUNCTIONS ##########
def read_scan_from_txt(filename):
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

########## CLASSES ##########
class RPLidar_sim(object):
	'''Class for communicating with RPLidar_sim rangefinder scanners : sampling time = 5seconds'''

	def __init__(self, filename, angle_step=1,DMAX=100, IMIN=0, IMAX=50):
		'''Initilize RPLidar_sim object for communicating with the sensor.

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
		scans_dict_list= read_scan_from_txt(self.filename)
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
				List of the measures. Each measurement is tuple with following
				format: (quality, Azimuth , distance). For values description please
				refer to `iter_measures` method's documentation.
		'''
		scans_dict_list= read_scan_from_txt(self.filename)
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

	
class RPLidar_sensor(object):
	'''Class for communicating with RPLidar_sim rangefinder scanners : sampling time = 5seconds'''

	def __init__(self, root, angle_step=1,DMAX=200, IMIN=0, IMAX=50):
		'''Initilize RPLidar_sim object for communicating with the sensor.

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
		self.root=root
		self.files_list=glob(os.path.join(root, 'missions', '*.json'))
		print(f'\n - {len(self.files_list)} Lidar files are found in : {root}')
		self.angle_step=angle_step
		self.DMAX=DMAX
		self.IMIN=IMIN
		self.IMAX=IMAX

	def plot_lidar(self, num_scans=100):
		
		scans_dict_list= self.load_mission_json()
		print(f'\n - Visualizing LiDAR data [{self.root}]\n - {len(scans_dict_list)} scans ae found!')

		join_distance_list =[]
		time_vec = []

		x = []
		y = []

		plt.ion()
		fig = plt.figure()

		for k,	scan in enumerate(scans_dict_list):
			input(f'\n flag: scan={scan}')
			detect_obj = scan['points']
			if len(detect_obj)<2:
				continue
			arr=np.array(detect_obj)
			x,y=arr[:,0]/10,arr[:,1]/10
			# sort y wrt x 
			x, y = zip(*sorted(zip(x, y)))
			# remove the offset_copy
			# y-= np.min(y)-1
			N,M=np.max(x), np.max(y)
			print(f'\n\n x={x} \n y={y}\n size={len(y)}\n')
			# input(f'\n  N={N}, M={M}')
			join_distance_list.append(detect_obj)
			time_vec.append(k)
			# plot
			plt.clf()
			ax = fig.add_subplot(111)
			ax.set_title(f'measurement={k}')
			ax.set_ylabel(' distance in (cm)')
			ax.set_ylim(0,self.DMAX)
			ax.set_xlabel(f'lane width (cm)')
			line1, = ax.plot(x, y, 'b-')
			line1.set_ydata(y)
			fig.canvas.draw()
			fig.canvas.flush_events()
			if k==num_scans:
				break

	def load_mission_json(self):
		import json
		scans_dict_list=[]
		# Strips the newline character
		for filename in self.files_list:
			# load the json file
			new_scan=utils.load_json(filename)
			scans_dict_list+=new_scan 
			input(f'\n flag: new_scan={new_scan}') 
		return scans_dict_list

class HAIS_visualizer(RPLidar_sensor):
	'''Class for communicating with RPLidar_sim range finder scanners : sampling time = 5seconds'''

	def __init__(self, root, disp=True, DMAX=100, refresh=30):
		self.root=root
		self.files_list=glob(os.path.join(self.root, 'missions', '*.json'))
		self.result_filepath=os.path.join(self.root, 'inspection_dic.json')
		self.disp=disp
		self.DMAX=DMAX
		self.refresh=refresh
		# plot
		if self.disp:
			plt.ion()
			self.fig = plt.figure()

	def get_full_path(self, filename):
		return os.path.join(self.root, filename) 

	def read_scan_file(self, filename):
		skip=False
		file_ID, ext = os.path.splitext(filename)
		try:
			if ext=='.json':# LiDAR data is  saved as json
				new_=utils.load_json(filename)
				if len(new_)==0:
					skip=True
				detect_obj = new_[0]['points']
			elif ext=='.npy': # LiDAR data is  saved as npy
				# new_=utils.load_json(filename)
				lidar_Y=np.load(filename)
				liday_X=np.array([k for k in range(lidar_Y.shape[0])])
				detect_obj=np.array(lidar_Y+liday_X)
			elif ext=='.txt':
				scans_dict_list = read_scan_from_txt(filename)
				detect_obj=np.array(scans_dict_list)
		except:
			print(f'\n Error: cannot read correctly the LiDAR file [{filename}]')

		return detect_obj, skip
	
	def visualize(self):
		scans_dict_list= self.load_mission_json()	
		Kinematics_list=[]
		lat_list, lon_list, alt_list, metric_list =[],[],[], []
		print(f'\n - Loading the collected data [{len(scans_dict_list)} samples]. Please wait ...')

		for k,	scan in enumerate(tqdm(scans_dict_list)):
			if k%self.refresh==0:
					plt.clf()
			if scan!={}:
				# LIDAR
				filename=self.get_full_path(scan['filename'])
				
				if scan['sensor_name']=='LIDAR':
					detect_obj, skip=self.read_scan_file()
					if skip:
						continue

					if self.disp:
						self.plot_lidar_data(detect_obj)

				elif 'CAMERA' in scan['sensor_name']:
					import cv2 
					try:
						img=cv2.imread(filename)
					except Exception as e:
						print(f'\n - Error: with in loading the image file: \n {filename}\n Exception: {e}')
						continue
					try:
						if self.disp:
							self.update_plot_camera(img)
					except Exception as e:
						print(f'\n error with the image: {img} of file {filename}\n Exception: {e}')
						continue

				elif scan['sensor_name']=='IMU_SENSOR':
					new_=scan['meta_data']
					try:
						[lat, lng, alt]=scan['position']['Translation']
					except Exception as e:
						print(f'\n Exception: {e}')
						continue
					# calibrate ofset
					# offset=[37.82922, -35.33007, 0]
					offset=[37.81533, -35.82142]
					lat+=offset[0]
					lng+=offset[1]
					# normlizatoin
					lat, lng, alt=lat/100, lng/100, alt/100
					car_location=[lat, lng, alt]
					lat_list.append(lat); lon_list.append(lng); alt_list.append(alt); metric_list.append(-1)

  				# Kinemetics
					AccXangle, AccYangle=new_['ACCX Angle'], new_['ACCY Angle']
					gyroXangle, gyroYangle, gyroZangle=new_['GRYX Angle'], new_['GYRY Angle'], new_['GYRZ Angle']
					CFangleX, CFangleY = new_['CFangleX Angle'], 0 #new_['CFangleY Angle'],
					heading, tiltCompensatedHeading = new_['HEADING'], new_['tiltCompensatedHeading']
					kalmanX, kalmanY= new_['kalmanX'], new_['kalmanY']
					Kinematics_list.append([AccXangle, AccYangle, gyroXangle, gyroYangle, gyroZangle, CFangleX, CFangleY, heading, tiltCompensatedHeading, kalmanX, kalmanY])
					if self.disp:
						self.plot_Kinemetics(Kinematics_list[-20:])

				else:
					continue
				
				# update the plot
				if self.disp:
					self.fig.canvas.draw()
					self.fig.canvas.flush_events()
			  
				# sleep  to see the plot
				time.sleep(0.5)

		# # update json
		# inspection_dict={'lon': lon_list, 'lat':  lat_list, 'alt':  alt_list, 'metric': metric_list}
		# utils.save_json([inspection_dict], self.result_filepath)

	def update_plot_camera(self, img):
		# plt.clf()
		## LIDAR
		ax1 = self.fig.add_subplot(223)
		ax1.clear()
		ax1.set_title(f'ROAD PICTURE [CAM]')
		ax1.imshow(img)
		ax1.set_axis_off()

	def plot_lidar_data(self, detect_obj):
		if len(detect_obj)<2:
			return True
		# print(f'\n  lidar scan={detect_obj}')
		arr=np.array(detect_obj)
		x,y=arr[:,0]/10,arr[:,1]/10
		# sort y wrt x 
		x, y = zip(*sorted(zip(x, y)))
		# remove the offset_copy
		y-= np.min(y)-1
		N,M=np.max(x), np.max(y)
		# plot
		self.update_plot_lidar(x, y)

	def update_plot_lidar(self, liday_X, lidar_Y):
		# plt.clf()
		## LIDAR
		ax2 = self.fig.add_subplot(224)
		ax2.clear()
		ax2.set_title(f'ROAD SURFACE [LIDAR]')
		ax2.set_ylabel(' distance in (cm)')
		ax2.set_ylim(0, 1.2*np.max(lidar_Y))#self.DMAX)
		# ax2.set_xlabel(f'lane width (cm)')
		line1, = ax2.plot(liday_X, lidar_Y, 'b-')
		line1.set_ydata(lidar_Y)
		plt.xticks([])

	def plot_Kinemetics(self, Kinematics_list):
		if len(Kinematics_list)==0:
			return True
		# print(f'\n  Kinemtics scan={Kinematics_list}')
		arr=np.array(Kinematics_list)
		AccXangle, AccYangle, gyroXangle, gyroYangle, gyroZangle, CFangleX, CFangleY, heading, tiltCompensatedHeading, kalmanX, kalmanY=\
		arr[:,0],arr[:,1],arr[:,2],arr[:,3],arr[:,4],arr[:,5],arr[:,6],arr[:,7],arr[:,8],arr[:,9],arr[:,10]
		X=[k for k in range(arr.shape[0]-1)]
		## Kinematics
		ax3 = self.fig.add_subplot(211)
		ax3.clear()
		ax3.set_title(f'VEHICULE KINEMATICS [IMU/GYRO/GPS]')
		# add plot
		# list_metrics=[AccXangle, AccYangle, gyroXangle, gyroYangle, gyroZangle, CFangleX, CFangleY, heading, tiltCompensatedHeading, kalmanX, kalmanY]
		from numpy import diff
		list_metrics=[diff(AccXangle), diff(AccYangle), diff(gyroXangle), diff(gyroYangle), diff(gyroZangle), diff(CFangleX), diff(CFangleY), diff(heading), diff(tiltCompensatedHeading), diff(kalmanX), diff(kalmanY)]
		list_label=['AccXangle', 'AccYangle', 'gyroXangle', 'gyroYangle', 'gyroZangle', 'CFangleX', 'CFangleY', 'heading', 'tiltCompensatedHeading', 'kalmanX', 'kalmanY']
		linestyles=["-", "-", "--", "--", "--", "-.","-.", "-", "-", ":", ":"]
		linewidth_list=[2.0, 2.0, 1.5, 1.5, 1.5, 1.5, 1.2, 1.2, 1.0,1.0, 0.8, 0.8]
		for var, label, color, linestyle, linewidth in zip(list_metrics, list_label, colors_list[:len(list_metrics)], linestyles, linewidth_list):
			line, = ax3.plot(X, var, linestyle=linestyle, linewidth=1.5, color=color,  label=label)
			line.set_ydata(var)

		plt.legend(loc="upper left")
		plt.xticks([])

	def update_map(self, car_location):
		e=0
		# ax2 = self.fig.add_subplot(23)

##################  TEST SENSORS  ##################
def run_Lidar_sim():
	DMAX = 100
	IMIN = 0
	IMAX = 50
	# flag [todo]: check this testing
	filename='data/test/catalog_0_A1_R5_465_1.catalog.txt'
	lidar = RPLidar_sim(filename, angle_step=2, DMAX=DMAX, IMIN=IMIN, IMAX=IMAX)
	# vizualise lidar image
	print('Visualize RPLidar simulation measurement')
	lidar.plot_lidar()
	lidar.plot_lidar_animation()

def run_Lidar_HAIS():
	DMAX = 100
	IMIN = 0
	IMAX = 50
	root= '../data/download/node1'

	lidar = RPLidar_sensor(root)
	# vizualise lidar image
	lidar.plot_lidar()

def run_HAIS_visualizer():
	DMAX = 100
	IMIN = 0
	IMAX = 50
	root= '../data/download/node1'
	lidar = HAIS_visualizer(root, disp=True)
	# vizualise lidar image
	print('Visualize collected the mission ')
	lidar.visualize()

if __name__ == '__main__':
	# # run the lidar simulator using online dataset format
	# run_Lidar_sim()

	# run the lidar sensor of the HAIS dataset format
	run_Lidar_HAIS()

	# # run the IMU sensor of the HAIS dataset format
	# run_HAIS_visualizer()

