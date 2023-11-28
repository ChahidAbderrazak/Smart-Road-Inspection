import os
import pandas as pd
import numpy as np
import osmnx as ox
# ox.config(use_cache=True, log_console=True)
# ox.config(log_console=True, log_file=True, use_cache=True,
# 						data_folder='temp/data', logs_folder='temp/logs',
# 						imgs_folder='temp/imgs', cache_folder='temp/cache')

import json
import shutil


def get_data_folder():
	download_path=os.path.join(os.path.dirname(os.getcwd()),'data','download')
	if not os.path.exists(download_path):
			# get the parent root folder
			download_path=os.path.join(os.path.dirname(os.path.dirname(os.getcwd())),'data','download')
	return download_path

def load_json(filename):
	try:
		if os.path.exists(filename):
			with open(filename) as f:
				data = json.load(f)
			f.close()
			return data

		else:
			msg =f'\n\n Error: The JSON file <{filename}> cannot be found!!'
			raise Exception(msg)
			
	except:
		msg = f'\n\n Error: The JSON file <{filename}> cannot be read correctly!!'
		print(msg)
		raise ValueError(msg)

def save_json(json_string, filename):
	try:
		# Using a JSON string
		with open(filename, 'w') as outfile:
			json.dump(json_string, outfile, indent=2)
			return 0
	except Exception as e:
		print(f'\n\n - error in saving {filename}\n Exception: {e}')
		return 1

def clean_directory(DIR):
		if not os.path.exists(DIR):
				os.makedirs(DIR)
		else:
			shutil.rmtree(DIR) 
			os.makedirs(DIR)


from math import sin, cos, sqrt, asin, radians
def gps_location_distance( point1, point2):

	'''
	Calculate the great circle distance in meters between two points 
	on the earth (specified in decimal degrees)
	'''
	lat1 = point1[0]
	lon1 = point1[1]
	lat2 = point2[0]
	lon2 = point2[1]

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

def fuse_inspection_dict_maps(inspection_path, new_inspection_path, min_dist=1):
	inspection_dict=load_json(inspection_path)
	inspection_dict2=load_json(new_inspection_path)
	df_new = pd.DataFrame(inspection_dict2)

	df = pd.DataFrame(inspection_dict)
	map_center_point, radius=get_map_center_dist(df, df_new)
	#get the location graph
	print(f'\n --> creating/downloading Map:  \
	 \n\t - center= {map_center_point} \
	 \n\t - dist= {radius} km \
	 \nThis will take some minutes. Please wait :)...')
	graph = ox.graph_from_point(map_center_point, dist=radius, network_type="drive")
	nodes, edges = ox.graph_to_gdfs(graph)
	max_dist=0
	min_dist=np.inf
	for ind_new in df_new.index:
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

import pandas as pd
import geopandas as gd
from shapely.geometry import Point

def get_sensor_data_from_location(inspection_dict, picked_location, disp=False):

	df = pd.DataFrame(inspection_dict)
	df['id'] = [k for k in range(len(df))]
	df=df.rename(columns={"lon": "Longitude", "lat": "Latitude", "alt": "Altitude"}, errors="raise")
	gdf = gd.GeoDataFrame(	df, geometry=gd.points_from_xy(df.Longitude, df.Latitude))

	if disp:
		print( f'\n\n df row ={df.head()}')
		print( f'\n\n gdf row ={gdf.head()}')
	point=Point(picked_location['lon'], picked_location['lat'])
	def min_dist(point, gdf):
			gdf['Dist'] = gdf.apply(lambda row:  point.distance(row.geometry),axis=1)
			nearest_node = gdf.iloc[gdf['Dist'].argmin()]
			return nearest_node

	nearest_node = min_dist(point, gdf)
	point1=(picked_location['lon'], picked_location['lat'])
	point2=(nearest_node["Longitude"], nearest_node["Latitude"])
	distance=gps_location_distance(point1, point2)
	sample_token=nearest_node["token"].split('_f')[0]
	if disp:
		print( f'\n\n The closed point to  input location [{point}] is the node[ id={nearest_node["id"]} ]: distance={distance} m')
		print( f'\n{nearest_node} ')
		print(f'\n output sample_token={sample_token} ')
	return sample_token, distance 

from nuscenes.nuscenes import NuScenes
def search_node_in_DB(database_root, picked_location, max_dist=2, version='v1.0', disp=False):
	'''
	searching the closed node to the GPS location <picked_location> within <max_dist> radius in meters
	'''
	# initialization 
	dict_sensor={"camera": "", 
							"lidar": "", 
							"Kenematics": "", 
							"weather":'',
							}
	empty_db=True

	# search the exising nodes
	# list_inspections= glob(os.path.join(database_root,'*', 'inspection_dic.json'))
	list_inspections=[os.path.join(path, name) for path, subdirs, files in os.walk(database_root) for name in files if name=='inspection_dic.json']
	if disp: 
		print(f'\n\n database_root={database_root}')
		print(f'\n ->  {len(list_inspections)} nodes are found: \n{list_inspections}')

	# find the nearest node to the <picked_location>
	for file in list_inspections:
		inspection_dict=load_json(file)
		if disp: 
			print(f'\n searching in mission: {os.path.basename(os.path.dirname(file))}')
		token, distance = get_sensor_data_from_location(inspection_dict,	picked_location, disp=disp)
		if distance<=max_dist:
			file_json=file
			max_dist=distance
			sample_token=token
			empty_db=False
			print(f'\n - found candidate: sample_token{sample_token}')
	
	# reteibe the the sample data
	if not empty_db:
		if disp:
			print(f'\n - Loading the Nuscenes database ...')
		
		dataroot=os.path.dirname(file_json)

		def create_sensor_data_dict(sample_token, dataroot, version, disp=True):
			nusc = NuScenes(version=version, dataroot=dataroot, verbose=disp)
			my_sample = nusc.get('sample', sample_token)
			# print(f'\n\n ==> First sample of the scene: \n {my_sample}')
			sensors_data=my_sample['data']
			if disp:
				print(f'\n\n sensor data={sensors_data} ')
			dict_sensor={}
			for id in sensors_data.keys():
				token=sensors_data[id]
				sensor_data = nusc.get('sample_data', token)
				if 'CAM' in id:
						dict_sensor.update({"camera": os.path.join(dataroot,sensor_data['filename']) })
				elif 'IMU' in id:
					msg_str=''
					dic_IMU=sensor_data['meta_data']
					for parm in dic_IMU.keys():
						value=int(dic_IMU[parm])
						msg_str+=f'| {parm}={value} |'
					dict_sensor.update({"Kenematics": msg_str})
				elif 'LIDA' in id:
					dict_sensor.update({"lidar": os.path.join(dataroot,sensor_data['filename']) })
			return dict_sensor
		# fill the sensor data
		dict_sensor= create_sensor_data_dict(sample_token, dataroot, version, disp=disp)
	return dict_sensor

if __name__ == '__main__':
	# update inspection dict
	database_root='../data/download/node1' 	

	# full_inspection_dict_path= os.path.join(os.getcwd(),'dataabase', 'inspection_dic.json')
	# # update teh inspection database
	# fuse_inspection_dict_maps(inspection_path=full_inspection_dict_path, new_inspection_path=filename)


	picked_location={"lat":43.937092, "lon":-78.867443}
	picked_location={'lat': 43.94622906450355, 'lon': -78.89602554498407}
	max_dist=15
	dict_sensor=search_node_in_DB(database_root,	picked_location, max_dist=max_dist, disp=True)
	print(f'\n sensor dict={dict_sensor}')

 
  
