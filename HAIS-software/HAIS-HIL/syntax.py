import os, tqdm, sys
from glob import glob
import numpy as np
import matplotlib.pyplot as plt


from lib import utils

def update_plot_lidar( liday_X, lidar_Y):
	# plt.clf()
	## LIDAR
	fig=plt.figure()
	ax2 = fig.add_subplot(224)
	ax2.set_title(f'ROAD SURFACE [LIDAR]')
	ax2.set_ylabel(' distance in (cm)')
	# ax2.set_ylim(0, 1.2*np.max(lidar_Y))#self.DMAX)
	# ax2.set_xlabel(f'lane width (cm)')
	line1, = ax2.plot(liday_X, lidar_Y, 'b-')
	line1.set_ydata(lidar_Y)
	plt.xticks([])
	plt.show()

filename='/media/abdo2020/DATA1/data/labeled-dataset/HAIS-project/download/node6-JN/sweeps/LIDAR/1677605184.75_tmstmp_2023-02-28-12h-26min-24sec__LIDAR__0.npy'
file_ID, ext = os.path.splitext(filename)

print('ext', ext)
# if ext=='.npy':
# 	lidar_Y=np.load(filename)
# 	liday_X=np.array([k for k in range(lidar_Y.shape[0])])

# print(f'\n lidar_Y={lidar_Y}')
# update_plot_lidar( liday_X, lidar_Y)
# utils.show_image(np_array_vec, img_title='LiDAR data')
# root='/media/abdo2020/DATA1/data/raw-dataset/hais-node/2023-02-17/CS1_Oshawa-road'
# root='/home/abdo2020/Downloads/Jetson_ROS_data/Node2/mission_2023-02-26-15h-49min-07sec'
# path=get_sensor_filename(sensor_name='fg', frame=0,time_tag='', tag='')
# tsmp1=get_timestamp(path)

# path=get_sensor_filename(sensor_name='rt', frame=0,time_tag='', tag='')
# tsmp2=get_timestamp(path)

# print(f'tsmp1={tsmp1}, tsmp2={tsmp2}')
# print(f'comp={tsmp1>tsmp2}')

# path1='/media/abdo2020/DATA1/data/raw-dataset/hais-node/2023-02-17/CS1_Oshawa-road/sweeps/RIGHT_CAMERA/2023-02-17-17h-30min-51sec__RIGHT_CAMERA__2660.jpg'
# path2='/media/abdo2020/DATA1/data/raw-dataset/hais-node/2023-02-17/CS1_Oshawa-road/sweeps/RIGHT_CAMERA/2023-02-17-17h-30min-57sec__RIGHT_CAMERA__2666.jpg'
# tsmp2=get_timestamp(path2)
# tsmp1=get_timestamp(path1)
# print(f'tsmp1={tsmp1}, tsmp2={tsmp2}')
# print(f'comp={tsmp1<tsmp2}')
