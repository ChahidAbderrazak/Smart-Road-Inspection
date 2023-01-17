#!/usr/bin/env python3
'''Visualize the road conditions map'''
import os, sys
from turtle import color
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import matplotlib.animation as animation
import geopandas as gpd
import osmnx as ox
ox.config(use_cache=True, log_console=True)

try:
    from lib import utils
except:
    import utils

class HAIS_map(object):
	'''Class manaing the inspection visualization'''

	def __init__(self, place_coord, radius=1000, max_St = 20):

		self.place_coord=place_coord
		self.radius=radius
		self.route_list = []
		self.route_colors = []
		self.available_colors = ['red', 'salmon', 'orange', 'yellow', 'green']
		self.max_St = max_St               # maximun streets annotation to visualise (eg: -1 for all)
		# initializethe map  the graph
		self.initialize_location_map()

	def initialize_location_map(self):
		#get the location graph
		self.graph = ox.graph_from_point(self.place_coord, dist=self.radius)#, network_type="drive")
		self.origin = ox.nearest_nodes(self.graph, X=self.place_coord[0],Y=self.place_coord[1])
		# Retrieve nodes and edges
		self.nodes, self.edges = ox.graph_to_gdfs(self.graph)
		# self.list_noes_coord()
		#get building 
		self.buildings = ox.geometries_from_point(self.place_coord, dist=self.radius, tags={'building':True}) # Retrieve buildings from the area:
		# print(f'\n buildings.columns={buildings.columns}')
		# Retrieve parks
		self.leisure = ox.geometries_from_point(self.place_coord, dist=self.radius, tags={'leisure':True}) # fetch data
		self.parks = self.leisure[self.leisure["leisure"].isin(["pitch","park","playground"])] # select all park polygons
		# Get water bodies map of the New York City area
		self.water =  ox.geometries_from_point(self.place_coord, dist=self.radius, tags={"natural": "water"}) 
	
	def list_noes_coord(self):
		list_coord=[]	
		for node_id in list(self.graph.nodes):
			list_coord.append([self.graph.nodes[node_id]['x'], self.graph.nodes[node_id]['y']])
		
		input(f'\n - list_coord = {list_coord}')
		
	def viz_location_map(self):
		# Plot in map
		fig, self.ax = plt.subplots()#figsize=(12,8))

		# Plot the parks
		self.parks.plot(ax=self.ax, facecolor="green")

		# Plot the water
		self.water.plot(ax=self.ax, facecolor="blue")

		# Plot buildings
		self.buildings.plot(ax=self.ax, facecolor='silver', alpha=0.7)

		# Plot street edges
		self.edges.plot(ax=self.ax, linewidth=1.5, edgecolor='dimgray')
		# print(f'\n - {node} \n\n - {origin_edge}, \n coord_center={coord_center}')

	def get_road_color(self, road_metric):
		route_color = self.available_colors[road_metric]
		return route_color

	def show_streets_names(self):
		#- show steets names
		steets_names=[]
		# reteive the streets names
		for _, edge in self.edges.fillna('').iterrows():
			# limit number of street to show
			if edge['name'] not in steets_names:
				steets_names.append(edge['name'])
			if len(steets_names)>self.max_St or self.max_St==-1:
				break
			else:
				c = edge['geometry'].centroid
				text = edge['name']
				self.ax.annotate(text, (c.x, c.y), c='black', fontsize=8, weight='bold')
	
	def get_random_point(self):
		orig, dest = np.random.choice(self.graph.nodes(), 2)
		node_=self.graph.nodes[orig]; start_point=(node_['x'],node_['y'])
		node_=self.graph.nodes[dest]; dest_point=(node_['x'],node_['y'])
		return start_point, dest_point

	def generate_random_road_slices(self, Nroutes=5):
		import random
		list_coodinate = []
		for k in range(Nroutes):
			start_point, dest_point = self.get_random_point()
			# print(f'\n flag sim: start_point={start_point}')
			# input(f'\n flag sim: dest_point={dest_point}')
			# get the earest node to the locations
			start = ox.nearest_nodes(self.graph, X=start_point[0],Y=start_point[1])
			dest = ox.nearest_nodes(self.graph, X=dest_point[0],Y=dest_point[1])
			if start!=dest:
				# get random road conditions
				road_metric = random.randint(0, 4)
				list_coodinate.append((start, dest, road_metric))
		return list_coodinate

	def convert_coord_to_nodes(self, list_inspected_coord):
		import random
		list_coodinate = []

		new_portion=True
		for k in range(len(list_inspected_coord)-1):
			if new_portion:
				start_point=list_inspected_coord[k][0]
				road_metric=[]

			road_metric.append(list_inspected_coord[k][1])
			dest_point =list_inspected_coord[k+1][0]
			print(f'\n flag coord: start_point={start_point}')
			print(f'\n flag coord: dest_point={dest_point}')
			
			# get the earest node to the locations
			start = ox.nearest_nodes(self.graph, X=start_point[1],Y=start_point[0])
			dest = ox.nearest_nodes(self.graph, X=dest_point[1],Y=dest_point[0])
			 
			print(f'\n start={start}, dest={dest}')
			if start!=dest:
				list_coodinate.append((start, dest, int(np.mean(road_metric))) )
				new_portion=True
			else:
				new_portion=False
				print(f'\n error: the road represents same node [start=dest]')

				if len(list_coodinate)==1:
					list_coodinate+=list_coodinate
					
		return list_coodinate

	def get_roads_conditions(self, list_inspected_roads):
		import networkx as nx
		self.route_list, self.route_colors = [], []
		# route/color listing
		for orig, dest, road_metric in list_inspected_roads:
			# orig, dest = np.random.choice(self.graph.nodes(), 2)
			route = nx.shortest_path(self.graph, orig, dest, weight='length')
			if len(route)>0:
				self.route_list.append(route)
				route_color = self.get_road_color(road_metric)
				self.route_colors.append(route_color)#[route_color]*(len(route) - 1))
	
	def viz_road_conditions(self, list_inspected_roads):
		self.viz_location_map()
		self.show_streets_names()
		# get the list of road with their conditions
		self.get_roads_conditions(list_inspected_roads)
		# plot the list of roa in different colors
		self.ax.set_title('HAIS: Inspected roads')
		self.ax.set_xticks([]);self.ax.set_yticks([])
		ox.plot_graph_routes(self.graph, ax=self.ax, routes=self.route_list, edge_linewidth=1, node_size=0, \
			                   route_colors = self.route_colors, route_linewidths=3)
		

def generate_random_roads_ispection():
	place_coord = (43.94593458427975, -78.89566960105887) # Ontario Tech University
	radius = 1000  # meters
	max_St = -1
	# create visualization object
	map=HAIS_map(place_coord=place_coord, radius=radius,max_St=max_St)

	# genrate  Nroutes randomly inspected roads 
	Nroutes=2
	list_inspected_roads = map.generate_random_road_slices(Nroutes=Nroutes)
	input(f'\n flag: list_inspected_roads={list_inspected_roads}')
	# visualize the road conditions
	map.viz_road_conditions(list_inspected_roads)

#########################  folium Visualization ##############################
from pandas import *
import numpy as np
from matplotlib import cm
import folium
from folium import plugins
from IPython.display import HTML, display

def rectify_route_positions(points_list):
		return [(i,j) for (i,j) in points_list if i!=-1 and j!=-1]

def draw_polylines(points, metrics, map, available_colors, inspect_status, horison=2):
		# add ledend
		lgd_txt = '<span style="color: {col};">{txt}</span>'
		cur_metrics=np.unique(metrics)
		cur_metrics=[k for k in cur_metrics if k!=-1]
		print(f'\n flag: cur_metrics={cur_metrics}')
		for k in range(len(cur_metrics)):
				name, color = inspect_status[k], available_colors[k]
				if name =='' :
					continue
				fg = folium.FeatureGroup(name= lgd_txt.format( txt= name, col= color))
				map.add_child(fg)
		folium.map.LayerControl('topleft', collapsed= False).add_to(map)

		# Need to have a corresponding color for each point
		if len(np.unique(cur_metrics))> len(available_colors):
			print(f'\n error: The defined {len(available_colors)} colors are not enough for {len(np.unique(metrics))} inspection cases ')
			raise ValueError
		# add inspection labels
		i=0
		while i < len(metrics)-horison:
			metrics_list=metrics[i:i+horison]
			points_list=points[i:i+horison]
			points_list=rectify_route_positions(points_list)
			if metrics[i]==-1 or len(points_list) <2:
				# print(f'\n - the road segment line{i} is ignored !!!\n - color={metrics[i]} \n - points={points[i]}')
				i+=horison-1
				continue

			avg_metric=int( np.mean(metrics_list) )
			try:
				curr=available_colors[avg_metric]
			except Exception as e:
				print(f'\n avg_metric={avg_metric}')
				print(f'\n error in coloring the map segment !!!\n Exception={e}')
				sys.exit(0)
			# print(f'\n - points_list={points_list} ,  metrics_list={metrics_list} ---> {avg_metric}')
			line = folium.PolyLine(points_list, color=curr, weight=8, opacity=0.8)
			line.add_to(map)
			i+=horison-1

def open_map_html(path):
	import webbrowser
	import os
	path=path.replace('\\', '/')
	if path[0]=='.' or path[0]=='/':
		path=path[1:]
	abs_path='/'+path
	for suffix in [os.getcwd()]:
		path_=os.path.join(suffix, path)
		if os.path.exists(path_):
			abs_path=path_
			break
	# 1st method how to open html files in chrome using
	filename='file:///'+ abs_path
	webbrowser.open_new_tab(filename)

def visualize_map(list_missions, maps_root, show_lanemarker=False):
	# average status horison
	horison=2
	# define te colors and inspection status:
	available_colors = ['gray', 'red',  'orange',  'green'] # ['gray', 'red', 'coral', 'orange', 'aquamarine2', 'green']
	if show_lanemarker:
		inspect_status = ['Scanned lanemarkers','Low reflectivity',  'Meduim reflectivity', 'Good reflectivity']
	else:
		inspect_status = ['Scanned road','Bad roard', 'Meduim roard', 'Good roard']
	# load HAIS data
	lat_coord, lon_coord, metric_list=[],[],[]
	cnt=0
	print(f'\n - The map will show {len(list_missions)} missions. Please wait :) ...')
	for dataroot in list_missions:
		list_files=utils.getListOfFiles(dataroot, ext='.json', path_pattern='inspection_dic.json')
		for inspect_filename in list_files:
			# inspect_filename=os.path.join(dataroot, 'inspection_dic.json')
			if not os.path.exists(inspect_filename):
				print(f'\n Error: inspection file not found: \n {inspect_filename}')
				# sys.exit(0)
				continue
			else:
				# print(f'\n Loading the inspection_dict: \n {inspect_filename}')
				inspection_dict=utils.load_json(inspect_filename)
				try:
					inspection_dict=inspection_dict[0]
				except:
					print('The inspection_dict is loaded successsfully!!')

				# upadte the map routes
				lat_coord+=inspection_dict['lat']+[-1]
				lon_coord+=inspection_dict['lon']+[-1]
				if show_lanemarker:
					metric_list+=inspection_dict['Lanemarker']+[-1]
					# print('landmaker status: \n',inspection_dict['Lanemarker'] )
				else:
					metric_list+=inspection_dict['metric']+[-1]
				cnt+=1

	# define the html map filepath
	if cnt>1:
		map_path=os.path.join(maps_root, 'inspection_Map_all_node.html')
	else:
		map_path=os.path.join(maps_root, 'inspection_Map_' +os.path.basename(dataroot)+'.html')

	inspection_routes={	'lat':lat_coord,
											'lon':lon_coord,
											'metric':metric_list}

	df = DataFrame(inspection_routes)
	points = zip(df['lat'], df['lon'])
	points = list(points)
	metrics=df['metric'].values
	# intialize the map
	df2 = df.drop(df[(df.lat==-1) & (df.lon==-1)].index)
	ave_lt = sum(df2['lat'])/len(df2)
	ave_lg = sum(df2['lon'])/len(df2)
	myMap = folium.Map(location=[ave_lt, ave_lg], zoom_start=10) 
	# draw the routes
	print(f'\n metrics={np.unique(metrics)}')
	draw_polylines(points, metrics, myMap, available_colors, inspect_status, horison=horison)

	# save map to html file
	myMap.save(map_path)
	print(f'\n - The inspectin map is saved in : \n {map_path}')

	# open the map in the browser
	open_map_html(map_path)

#%%####################### MAIN 
def run_visualize_nodes_inspection():
	maps_root='bin/maps'
	##-------------------  DRONE -------------------
	# list_missions=['/media/abdo2020/DATA1/data/raw-dataset/hais-drone/inspection/2022-10-12/UIOT-bridge/bridge',
	# 							'/media/abdo2020/DATA1/data/raw-dataset/hais-drone/inspection/2022-10-12/road/ERC-parking']

	##-------------------  NODE -------------------
	list_missions=[	'/media/abdo2020/DATA1/data/raw-dataset/hais-node/2022-10-11/UOIT-parking-Abderrazak',
									'/media/abdo2020/DATA1/data/raw-dataset/hais-node/2022-10-12/Oshawa-roads_all',
									'/media/abdo2020/DATA1/data/raw-dataset/hais-node/2022-10-31/HAIS_DATABASE-medium-speed',
									'/media/abdo2020/DATA1/data/raw-dataset/hais-node/2022-10-31/HAIS_DATABASE-high-speed' ,
									'/media/abdo2020/DATA1/data/raw-dataset/hais-node/2022-12-12/road-and-mark',
									'/media/abdo2020/DATA1/data/raw-dataset/data-demo/HAIS-data/testing_node2']
	
	
	# save the visualized map
	visualize_map(list_missions, maps_root=maps_root)


											
if __name__ == '__main__':

	# # genrate  Nroutes randomly inspected roads 
	# generate_random_roads_ispection()

	# Visuaise a list of inspected roads
	run_visualize_nodes_inspection()




