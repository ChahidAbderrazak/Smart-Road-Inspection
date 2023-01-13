from lib import hais_database, dji_drone, utils
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import osmnx as ox
# ox.config(use_cache=True, log_console=True)
ox.config(log_console=True, log_file=True, use_cache=True,
						data_folder='temp/data', logs_folder='temp/logs',
						imgs_folder='temp/imgs', cache_folder='temp/cache')
def deblur_gray_image(imblur):
	import pylops
	Nz, Nx = imblur.shape

	# Blurring guassian operator
	nh = [15, 25]
	hz = np.exp(-0.1 * np.linspace(-(nh[0] // 2), nh[0] // 2, nh[0]) ** 2)
	hx = np.exp(-0.03 * np.linspace(-(nh[1] // 2), nh[1] // 2, nh[1]) ** 2)
	hz /= np.trapz(hz)  # normalize the integral to 1
	hx /= np.trapz(hx)  # normalize the integral to 1
	h = hz[:, np.newaxis] * hx[np.newaxis, :]

	# fig, ax = plt.subplots(1, 1, figsize=(5, 3))
	# him = ax.imshow(h)
	# ax.set_title("Blurring operator")
	# fig.colorbar(him, ax=ax)
	# ax.axis("tight")
	Cop = pylops.signalprocessing.Convolve2D(
			(Nz, Nx), h=h, offset=(nh[0] // 2, nh[1] // 2), dtype="float32"
	)


	# imblur = Cop * im

	imdeblur = pylops.optimization.leastsquares.normal_equations_inversion(
			Cop, imblur.ravel(), None, maxiter=50  # solvers need 1D arrays
	)[0]
	imdeblur = imdeblur.reshape(Cop.dims)

	Wop = pylops.signalprocessing.DWT2D((Nz, Nx), wavelet="haar", level=3)
	Dop = [
			pylops.FirstDerivative((Nz, Nx), axis=0, edge=False),
			pylops.FirstDerivative((Nz, Nx), axis=1, edge=False),
	]
	DWop = Dop + [Wop]

	imdeblurfista = pylops.optimization.sparsity.fista(
			Cop * Wop.H, imblur.ravel(), eps=1e-1, niter=100
	)[0]
	imdeblurfista = imdeblurfista.reshape((Cop * Wop.H).dims)
	imdeblurfista = Wop.H * imdeblurfista

	imdeblurtv = pylops.optimization.sparsity.splitbregman(
			Cop,
			imblur.ravel(),
			Dop,
			niter_outer=10,
			niter_inner=5,
			mu=1.5,
			epsRL1s=[2e0, 2e0],
			tol=1e-4,
			tau=1.0,
			show=False,
			**dict(iter_lim=5, damp=1e-4)
	)[0]
	imdeblurtv = imdeblurtv.reshape(Cop.dims)

	imdeblurtv1 = pylops.optimization.sparsity.splitbregman(
			Cop,
			imblur.ravel(),
			DWop,
			niter_outer=10,
			niter_inner=5,
			mu=1.5,
			epsRL1s=[1e0, 1e0, 1e0],
			tol=1e-4,
			tau=1.0,
			show=False,
			**dict(iter_lim=5, damp=1e-4)
	)[0]
	imdeblurtv1 = imdeblurtv1.reshape(Cop.dims)

	return imdeblur, imdeblurfista, imdeblurtv, imdeblurtv1

def visualize_deblured_images(imblur, imdeblur, imdeblurfista, imdeblurtv, imdeblurtv1):
	# Nz, Nx = imblur.shape
	# sphinx_gallery_thumbnail_number = 2
	fig = plt.figure(figsize=(12, 6))
	fig.suptitle("Deblurring", fontsize=14, fontweight="bold", y=0.95)
	ax1 = plt.subplot2grid((2, 3), (0, 0))
	ax2 = plt.subplot2grid((2, 3), (0, 1))
	ax3 = plt.subplot2grid((2, 3), (0, 2))
	ax4 = plt.subplot2grid((2, 3), (1, 0))
	ax5 = plt.subplot2grid((2, 3), (1, 1))
	ax6 = plt.subplot2grid((2, 3), (1, 2))
	# ax7 = plt.subplot2grid((2, 5), (0, 3), colspan=2)
	# ax8 = plt.subplot2grid((2, 5), (1, 3), colspan=2)
	# ax1.imshow(img, cmap="viridis", vmin=0, vmax=250)
	# ax1.axis("tight")
	# ax1.set_title("Original")
	ax2.imshow(imblur, cmap="viridis", vmin=0, vmax=250)
	ax2.axis("tight")
	ax2.set_title("Blurry")
	ax3.imshow(imdeblur, cmap="viridis", vmin=0, vmax=250)
	ax3.axis("tight")
	ax3.set_title("Deblurred")
	ax4.imshow(imdeblurfista, cmap="viridis", vmin=0, vmax=250)
	ax4.axis("tight")
	ax4.set_title("FISTA deblurred")
	ax5.imshow(imdeblurtv, cmap="viridis", vmin=0, vmax=250)
	ax5.axis("tight")
	ax5.set_title("TV deblurred")
	ax6.imshow(imdeblurtv1, cmap="viridis", vmin=0, vmax=250)
	ax6.axis("tight")
	ax6.set_title("TV+Haar deblurred")

	# ax7.plot(im[Nz // 2], "k")
	# ax7.plot(imblur[Nz // 2], "--r")
	# ax7.plot(imdeblur[Nz // 2], "--b")
	# ax7.plot(imdeblurfista[Nz // 2], "--g")
	# ax7.plot(imdeblurtv[Nz // 2], "--m")
	# ax7.plot(imdeblurtv1[Nz // 2], "--y")
	# ax7.axis("tight")
	# ax7.set_title("Horizontal section")
	# ax8.plot(im[:, Nx // 2], "k", label="Original")
	# ax8.plot(imblur[:, Nx // 2], "--r", label="Blurred")
	# ax8.plot(imdeblur[:, Nx // 2], "--b", label="Deblurred")
	# ax8.plot(imdeblurfista[:, Nx // 2], "--g", label="FISTA deblurred")
	# ax8.plot(imdeblurtv[:, Nx // 2], "--m", label="TV deblurred")
	# ax8.plot(imdeblurtv1[:, Nx // 2], "--y", label="TV+Haar deblurred")
	# ax8.axis("tight")
	# ax8.set_title("Vertical section")
	# ax8.legend(loc=5, fontsize="small")

	plt.tight_layout()
	plt.subplots_adjust(top=0.8)
	plt.show()


def load_json(filename):
	try:
		if os.path.exists(filename):
			import json
			with open(filename) as f:
				data = json.load(f)
			f.close()
			return data

		else:
			msg =f'\n\n Error: The JSON file <{filename}> cannot be cound!!'
			raise Exception(msg)
			
	except:
		msg = f'\n\n Error: The JSON file <{filename}> cannot be read correctly!!'
		print(msg)
		raise ValueError(msg)


def gps_location_distance( point1, point2):
	from math import sin, cos, sqrt, asin, radians
	lat1 = point1[0]
	lon1 = point1[1]
	lat2 = point2[0]
	lon2 = point2[1]

# from math import radians, cos, sin, asin, sqrt
	"""
	Calculate the great circle distance in kilometers between two points 
	on the earth (specified in decimal degrees)
	"""
	# convert decimal degrees to radians 
	lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])

	# haversine formula 
	dlon = lon2 - lon1 
	dlat = lat2 - lat1 
	a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
	c = 2 * asin(sqrt(a)) 
	r = 6371 # Radius of earth in kilometers. Use 3956 for miles. Determines return value units.
	return c * r *1000

def get_map_center_dist(df, df_new):
	ave_lt = int( 10e4*sum(df['lat'])/len(df) )/10e4
	ave_lg = int( 10e4*sum(df['lon'])/len(df) )/10e4
	map_center_point=(ave_lt, ave_lg)
	df_fuse=pd.concat([df, df_new], ignore_index=True, sort=False)
	max_dist=0
	for ind_new in df_fuse.index:
		point_new=(df_fuse['lat'][ind_new], df_fuse['lon'][ind_new])
		#Compute the distance in meters
		distance = gps_location_distance( point_new, map_center_point)
		if max_dist<distance:
			max_dist=distance
	return map_center_point, 100+int(distance/1000)

def update_inspection_dict(inspection_path, new_inspection_path, min_dist=1):
	inspection_dict=load_json(inspection_path)
	inspection_dict2=load_json(new_inspection_path)
	df_new = pd.DataFrame(inspection_dict2)

	df = pd.DataFrame(inspection_dict)
	map_center_point, radius=get_map_center_dist(df, df_new)
	#get the location graph
	print(f'\n --> creating/downloading Map:  \
	 \n\t - center= {map_center_point} \
	 \n\t - dist= {radius} km \
	 \ntHis will take some minutes. Please wait :)...')
	graph = ox.graph_from_point(map_center_point, dist=radius, network_type="drive")
	nodes, edges = ox.graph_to_gdfs(graph)
	max_dist=0
	min_dist=np.inf
	for ind_new in df_new.index[:5]:
		point_new=(df_new['lat'][ind_new], df_new['lon'][ind_new])
		# find the closed node
		osmid = ox.nearest_nodes(graph, X=point_new[0], Y=point_new[1])
		closest_node=nodes.loc[osmid]#[nodes['osmid']==osmid]
		point_= (closest_node['y'], closest_node['x'])
		distance = gps_location_distance( point_new, point_)
		if max_dist<distance:
			max_dist=distance
		if min_dist>distance:
			min_dist=distance	
		# print(f'\n\n - point_new={point_new} --> {point_} \n - distance={distance}')
		# print(f'\n - closest_node [osmid={osmid}]: \n{closest_node}')
	print(f'\n\n - max_dist={max_dist} m , min_dist={min_dist} m\n - distance={distance} m')

if __name__ == '__main__':
	# # img_path='/media/abdo2020/DATA1/data/raw-dataset/data-demo/road-conditions-google/good-roads/road1.jpeg'
	# img_path='/media/abdo2020/DATA1/data/raw-dataset/hais-node/2022-10-31/HAIS_DATABASE-medium-speed/sweeps/CSI_CAMERA/2022-10-31-17h-32min-16sec__CSI_CAMERA__1.jpg'
	# imblur =  utils.load_image(img_path)

	# imdeblur=imdeblurfista=imdeblurtv=imdeblurtv1=np.empty_like(imblur)
	# #  Debluring
	# print(f'\n\n ==> Image debluring ... \n ')

	# for k in range(3):
	# 	print('.')
	# 	im =  imblur[:,:,k]
	# 	imdeblur[:,:,k], imdeblurfista[:,:,k], imdeblurtv[:,:,k], imdeblurtv1[:,:,k]=deblur_gray_image(im)
	# visualize_deblured_images(imblur, imdeblur, imdeblurfista, imdeblurtv, imdeblurtv1)

	# update inspection dict
	filename0='/media/abdo2020/DATA1/data/raw-dataset/hais-node/2022-10-31/HAIS_DATABASE-high-speed/inspection_dic.json' 
	filename1='/media/abdo2020/DATA1/data/raw-dataset/hais-node/2022-12-12/road-and-mark/inspection_dic.json' #
	# filename1='/media/abdo2020/DATA1/data/raw-dataset/hais-node/2022-10-31/HAIS_DATABASE-medium-speed/inspection_dic.json' 

	update_inspection_dict(inspection_path=filename0, new_inspection_path=filename1)


	# import pandas as pd
	# import geopandas
	# import matplotlib.pyplot as plt
	# import geopandas as gd
	# from shapely.geometry import Point
	# from shapely.ops import nearest_points
	# df = pd.DataFrame(inspection_dict)
	# df['id'] = [k for k in range(len(df))]
	# df=df.rename(columns={"lon": "Longitude", "lat": "Latitude", "alt": "Altitude"}, errors="raise")
	# df1 = gd.GeoDataFrame(	df, geometry=gd.points_from_xy(df.Longitude, df.Latitude))
	# df1['centroid'] = df1.centroid
	# print( f'\n map has {len(df)} nodes:\n', df1.head() )

	# df = pd.DataFrame(inspection_dict2)
	# df=df.rename(columns={"lon": "Longitude", "lat": "Latitude"}, errors="raise")
	# df2 = gd.GeoDataFrame(	df, geometry=gd.points_from_xy(df.Longitude, df.Latitude))
	# df2['centroid'] = df2.centroid

 


