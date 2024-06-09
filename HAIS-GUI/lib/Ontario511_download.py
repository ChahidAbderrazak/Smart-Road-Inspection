import os, sys
import cv2
import json, time
import signal
from tqdm import tqdm
import urllib.request
from datetime import datetime
import matplotlib.pyplot as plt
import polyline
from timeit import default_timer as timer


def timeout_handler(num, stack):
    print("\n\n - Raise Timeout flag!!")
    raise Exception("FUBAR")

signal.signal(signal.SIGALRM, timeout_handler)

class Ontario511(): 
	def __init__(self, dst_root='Ontario511', fs=300): 
		super().__init__()
		# parameters
		self.dst_root=dst_root    									# destination folder
		self.fs=fs													# download repetition frequency f Cameras frames

	def get_cameras(self):
		cam_api_url='https://511on.ca/api/v2/get/cameras'
		string= urllib.request.urlopen(cam_api_url).read().decode('utf-8')
		cam_dic = json.loads(string)
		print(f'\n- {len(cam_dic)} road cameras are found!')
		return cam_dic

	def get_time_tag(self, type=1):
		today = datetime.now()
		if type==0:
				return today.strftime("__%Y-%m-%d")
		else:
				return today.strftime("%Y-%m-%d-%Hh-%Mmin-%Ssec")

	def load_image(self, path):
		
		# print(' The selected image is :', path)
		# filename, file_extension = os.path.splitext(path)
		try:
				img = cv2.imread(path)
				# print(f' The selected image file is [{path}] of size {img.shape}')
				return 0, img
		except Exception as e :
				msg = '\n Error: the image path ' + path + f'cannot be loaded!!!!\n Exception: {e}'
				print(msg)
				return 1, []

	def show_image(self, img, img_title, cmap="cividis", figsize = (8,8)):
		# show image
		fig = plt.figure(figsize = figsize) # create a 5 x 5 figure 
		ax3 = fig.add_subplot(111)
		ax3.imshow(img, interpolation='none', cmap=cmap)
		ax3.set_title(img_title)#, fontsize=40)
		# plt.savefig('./residual_image.jpg')   
		plt.axis("off")
		plt.show()
	
	def create_new_folder(self, DIR):
		if not os.path.exists(DIR):
			os.makedirs(DIR)

	def save_json(self, data_dict, filepath):
		# save to .json
		self.create_new_folder(os.path.dirname(filepath))
		if not os.path.exists(filepath):
			with open(filepath, "w") as outfile:
				json.dump(data_dict, outfile, indent=2)
				
	def download_CAM_frame(self, url, filePath, disp=False):
		global camera_dict

		cam_frame = urllib.request.urlopen(url).read()
		# print("downloading: ",url)
		# print (filePath)
		corrp_img=cam_frame[0]==137
		# print(f'\, cam_frame={cam_frame[0]} \n corrupted={corrp_img}')
		# save uncorrupted images
		if not corrp_img:
			self.create_new_folder(os.path.dirname(filePath))
			with open(filePath, 'wb') as localFile:
				localFile.write(cam_frame)

				# save camera info in json
				camera_info=os.path.join(os.path.dirname(filePath), 'camera_info.json' )
				self.save_json(camera_dict, camera_info)

				# display:
				if disp:
					err1, img = self.load_image(filePath)
					# show image
					self.show_image(img, img_title=f'{os.path.basename(filePath)} [error={corrp_img}]')
			localFile.close()

	def download_Ontario511_CAM(self, disp=False):
		global camera_dict
		# get the list of camera dict	
		cam_dic=self.get_cameras()

		# downloading all images
		for camera_dict in tqdm(cam_dic[0:-1]):
			try:
				# get the camera info
				cam_enabled=camera_dict["Status"] 
				if cam_enabled=="Enabled":
					url=camera_dict["Url"]
					camera_ID=camera_dict["Id"]
					RoadwayName=camera_dict['RoadwayName']
					timestamp=self.get_time_tag(type=1)

					if RoadwayName != None:
						filePath = os.path.join(self.dst_root, "CAMERAS", camera_dict['RoadwayName'], camera_ID, timestamp+'.jpg')
						# download/save camera framess
						self.download_CAM_frame(url, filePath, disp=disp)

			except Exception as e:
				print(f'\n - warring: error in downloading the camera {camera_ID} \n \tException: {e}')
				# print(f'\n camera_dict={camera_dict}')

			except KeyboardInterrupt:
				print(f'\n - Exit Ontario511 download!')
				sys.exit(0)

	def download_road_conditions(self, disp=False):
		global camera_dict
		url='https://511on.ca/api/v2/get/roadconditions'
		string= urllib.request.urlopen(url).read().decode('utf-8')
		road_dict = json.loads(string)
		print(f'\n - {len(road_dict)} road condition are found !!! \n - destination= {self.dst_root}')

		# downloading all images
		for data_dict in tqdm(road_dict[0:-1]):
			try:
				LastUpdated=data_dict["LastUpdated"]
				# Converting timestamp to DateTime object
				datetime_object = str(datetime.datetime.fromtimestamp(LastUpdated))
				data_dict["DateTime"]=datetime_object
				RoadwayName=data_dict["RoadwayName"]
				Condition=data_dict["Condition"]
				road_class="__".join(Condition)
				road_class=road_class.replace('  ', ' ')
				road_class=road_class.replace(' ', '-')
				# remove the EncodedPolyline
				if 'EncodedPolyline' in data_dict.keys():
					route_codeded=data_dict["EncodedPolyline"]
					# input(f'\n - route_codeded={route_codeded}')
					data_dict.pop("EncodedPolyline")
					positions=polyline.decode(route_codeded)
					
					data_dict["positions"]=positions
					# input(f'\n - positions={positions}')
				# save road condition in json
				if RoadwayName != None:
					filePath = os.path.join(self.dst_root, "road-conditions", road_class, RoadwayName, str(LastUpdated) + '.json')
					self.save_json(data_dict, filePath)
					
			except Exception as e:
				print(f'\n - warring: error in downloading the conditions of the road [{RoadwayName}] \n \tException: {e}')
				print(f'\n data_dict={data_dict}')
				sys.exit(0)

			except KeyboardInterrupt:
				print(f'\n - Exit Ontario511 download!')
				sys.exit(0)
	def download(self, disp=False):		
		# downloading all images
		cnt=0
		frame_fs=self.fs
		signal.alarm(3*frame_fs)
		while True:
			# elapsed time 
			start = timer()
			# download the frame
			try:
				cnt+=1
				######## download CAM streaming ######
				print(f'\n ------------ Download Camera frame: {cnt} ------------')
				self.download_Ontario511_CAM(disp=disp)

				######## download HW conditions ######
				print(f'\n ------------ Download road condition: {cnt} ------------')
				self.download_road_conditions(disp=disp)

				# exit the code after a specific duration
				estimated_frame_timeout = timer()-start
				
			except Exception as e:
				if "FUBAR" in str(e):
					print("\n - Frame Time out! \n Exiting the program forcedly")
					sys.exit()
					
			
			finally:
				print(f"\n - Frame Download duration={estimated_frame_timeout} sec]")
			# sleep
			print(f'\n ------------ Sleeping [ {int(self.fs/60)} min] ------------')
			time.sleep(frame_fs)	
			# time out flag
			signal.alarm(int(3*estimated_frame_timeout))


if __name__ == '__main__':
	root=os.path.join(os.path.dirname(os.getcwd()),'data')
	print(f'root1={root}')
	if not os.path.exists(root):
		# get the parent root folder
		root=os.path.join(os.path.dirname(os.path.dirname(os.getcwd())),'data')
		print(f'root2={root}')
		

	dst_root=os.path.join(root, 'raw-data','Ontario511')
	print(f'\n - Downloading the data to : {dst_root}')
	# instantiate  Ontario511()
	ont511= Ontario511(dst_root)
	ont511.download(disp=False)
