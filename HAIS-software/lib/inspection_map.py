#!/usr/bin/env python3
'''Visualize the road conditions map'''
from turtle import color
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import matplotlib.animation as animation
import geopandas as gpd
import osmnx as ox
ox.config(log_console=True, use_cache=True)

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
			print(f'\n flag sim: start_point={start_point}')
			input(f'\n flag sim: dest_point={dest_point}')
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
from IPython.display import HTML, display

def draw_polylines(points, metrics, map, horison=2):
		available_colors = ['red', 'coral', 'orange', 'aquamarine2', 'green']
		inspect_status = ['Bad roard', '', 'Meduim roard', '', 'Good roard']
		# add ledend
		lgd_txt = '<span style="color: {col};">{txt}</span>'
		for name, color in zip(inspect_status, available_colors):  
				if name =='':
					continue
				fg = folium.FeatureGroup(name= lgd_txt.format( txt= name, col= color))
				map.add_child(fg)
		folium.map.LayerControl('topleft', collapsed= False).add_to(map)

		# Need to have a corresponding color for each point
		if len(np.unique(metrics))> len(available_colors):
			print(f'\n error: The defined {len(available_colors)} colors are not enough for {len(np.unique(metrics))} inspection cases ')
			raise ValueError
		# add inspection labels
		i=0
		while i < len(metrics)-horison:
			metrics_list=metrics[i:i+horison]
			points_list=points[i:i+horison]
			avg_metric=int( np.mean(metrics_list) )
			curr=available_colors[avg_metric]
			# print(f'\n - points_list={points_list} ,  metrics_list={metrics_list} ---> {avg_metric}')
			line = folium.PolyLine(points_list, color=curr, weight=10.5, opacity=1)
			line.add_to(map)
			i+=horison-1



def visualize_map(inspection_dict, map_path):
	"""	
	inspection_dict={'lon': [-78.8977848, -78.8980208, -78.8992868, -78.8996731, -78.8961326, -78.892592, -78.8914548, -78.8877212, -78.8840948], 
									'lat': [43.9455189, 43.9461678, 43.9462296, 43.9472801, 43.9480217, 43.9454571, 43.9446692, 43.9420581, 43.9395551],
									'metric': [ 0, 0, 2, 2, 2, 1, 4, 4, 2]
									}
	"""

	df = DataFrame(inspection_dict)
	ave_lt = sum(df['lat'])/len(df)
	ave_lg = sum(df['lon'])/len(df)
	points = zip(df['lat'], df['lon'])
	points = list(points)
	myMap = folium.Map(location=[ave_lt, ave_lg], zoom_start=10) 
	draw_polylines(points, df['metric'].values, myMap)
	# save map to html file
	myMap.save(map_path)


def visualize_list_roads_ispection():
	# demo
	inspection_dict={'lon': [-78.8977848, -78.8980208, -78.8992868, -78.8996731, -78.8961326, -78.892592, -78.8914548, -78.8877212, -78.8840948], 
									'lat': [43.9455189, 43.9461678, 43.9462296, 43.9472801, 43.9480217, 43.9454571, 43.9446692, 43.9420581, 43.9395551],
									'metric': [ 0, 0, 2, 2, 2, 1, 4, 4, 2]
									}
	# Hais node 2022-10-12
	inspection_dict={'lat': [43.9455189, 44.950747, 44.950774, 44.9507526, 44.9507211, 44.9503352, 44.9502605, 44.9501825, 44.9500171, 44.9499311, 44.9497545, 44.9495761, 44.9488408, 44.9485513, 44.9476621, 44.9475642, 44.9474665, 44.9467789, 44.9466834, 44.946588199999994, 44.946303900000004, 44.9461141, 44.9460192, 44.9459245, 44.9458314, 44.94564610000001, 44.94517909999999, 44.945085799999994, 44.9448975, 44.94480359999999, 44.9440532, 44.9439598, 44.94386599999999, 44.9436789, 44.9435855, 44.9433071, 44.943033699999994, 44.9427632, 44.9426732, 44.9425832, 44.9424933, 44.9421313, 44.9417618, 44.9415728, 44.9412837, 44.941187, 44.9408964, 44.9407046, 44.9404252, 44.9403327, 43.5997915, 43.599704, 43.5993544, 43.5984403, 43.5981632, 43.5978843, 43.597791, 43.597033499999995, 43.5966455, 43.5965477, 43.596052799999995, 43.595852400000005, 43.5954468, 43.5949376, 43.594837, 43.5946378, 43.5938768, 43.5937064, 43.5936227, 43.592891099999996, 43.5926614, 43.5922906, 43.592216099999995, 43.5920671, 43.591600799999995, 43.591519500000004, 43.5912011, 43.5911244, 43.5910475, 43.5908983, 43.5908242, 43.5906706, 43.5905946, 43.59035050000001, 43.59018269999999, 43.5900997, 43.590018300000004, 43.589939, 43.589788799999994, 43.5897176, 43.5896493, 43.5895804, 43.588874100000005, 43.5888079, 43.5887406, 43.5883251, 43.5881099, 43.5878248, 43.5874831, 43.5874141, 43.5872744, 43.5872026, 43.586658, 43.586492099999994, 43.5861534, 43.5860675, 43.585808500000006, 43.585553600000004, 43.5844667, 43.5842802, 43.5838214, 43.583638799999996, 43.5820673, 43.5820033, 43.5818513, 43.5818113, 43.5817202, 43.5817172, 43.5816624, 43.5815398, 43.5814303, 43.581371, 43.5811793, 43.58110980000001, 43.5808223, 43.5807478, 43.5806014, 43.580529500000004, 43.5804598, 43.580330999999994, 43.580057100000005, 43.5799286, 43.5796944, 43.579531100000004, 43.579418, 43.5793626, 43.5793079, 43.5790812, 43.578768200000006, 43.5785727, 43.578386300000005, 43.57833109999999, 43.5782075, 43.5781348, 43.5781238, 43.5781228, 43.5781016, 43.578083500000005, 43.578059, 43.578043, 43.578006699999996, 43.5779921, 43.5779421, 43.577921399999994, 43.577888599999994, 43.577837099999996, 43.577835400000005, 43.5778516, 43.577455099999995, 43.57735219999999, 43.5768674, 43.5766852, 43.575876099999995, 43.5754649, 43.575291, 43.5752478], 
								'lon': [-78.8977848, -78.8958266, -78.8955192, -78.89549450000001, -78.8954766, -78.8954126, -78.8954041, -78.8953955, -78.8953749, -78.8953637, -78.8953393, -78.8953184, -78.89522880000001, -78.8951948, -78.8950048, -78.8949676, -78.8949261, -78.8946407, -78.8946011, -78.8945606, -78.8944396, -78.894361, -78.894321, -78.8942813, -78.89423950000001, -78.8941539, -78.8939572, -78.8939168, -78.8938369, -78.8937995, -78.8934858, -78.89344600000001, -78.8934064, -78.8933263, -78.893287, -78.8931718, -78.8930576, -78.892944, -78.8929068, -78.8928694, -78.8928316, -78.8926769, -78.8925244, -78.8924454, -78.8923221, -78.8922793, -78.8921478, -78.8920555, -78.8919005, -78.89184170000001, -78.8915019, -78.8914464, -78.8912299, -78.8907741, -78.8906514, -78.8905276, -78.8904853, -78.8901493, -78.5499801, -78.5499373, -78.5497203, -78.5496328, -78.5494546, -78.549232, -78.5491886, -78.5491019, -78.548764, -78.5486889, -78.5486527, -78.54832929999999, -78.548229, -78.5480642, -78.5480315, -78.5479671, -78.5477722, -78.5477358, -78.54759659999999, -78.5475641, -78.5475339, -78.5474713, -78.547438, -78.54737519999999, -78.5473379, -78.5472272, -78.5471471, -78.547107, -78.5470705, -78.5470356, -78.5469668, -78.5469354, -78.5469058, -78.5468752, -78.5465744, -78.5465427, -78.5465129, -78.5463353, -78.54624340000001, -78.5461113, -78.545957, -78.5459234, -78.545856, -78.5458227, -78.545567, -78.5454886, -78.5453351, -78.545295, -78.5451764, -78.5450621, -78.5446161, -78.54456350000001, -78.5444031, -78.5443372, -78.5437029, -78.5436698, -78.5435873, -78.54356539999999, -78.5435156, -78.5435114, -78.5434778, -78.5434089, -78.5433493, -78.543318, -78.5432186, -78.54318669999999, -78.5430544, -78.5430219, -78.54295619999999, -78.5429251, -78.542895, -78.5428374, -78.5427181, -78.5426646, -78.5425587, -78.54249010000001, -78.5424416, -78.5424174, -78.54239319999999, -78.5423017, -78.5421662, -78.5420957, -78.54202790000001, -78.5420072, -78.5419565, -78.541925, -78.5419216, -78.5419215, -78.54191259999999, -78.5419075, -78.5419074, -78.5419106, -78.5419327, -78.5419573, -78.542114, -78.5422102, -78.5423564, -78.542563, -78.5426691, -78.5427442, -78.5442101, -78.5446075, -78.5454227, -78.5453341, -78.5449816, -78.5447516, -78.5446613, -78.544641], 
								'metric': [2, 3, 0, 3, 0, 2, 3, 3, 3, 3, 3, 2, 2, 1, 0, 1, 2, 2, 2, 1, 0, 2, 1, 3, 2, 2, 0, 3, 3, 1, 2, 1, 0, 1, 2, 0, 3, 3, 3, 0, 2, 1, 2, 0, 3, 2, 2, 1, 2, 3, 1, 2, 1, 0, 1, 1, 3, 2, 2, 2, 1, 3, 0, 1, 3, 3, 0, 3, 2, 3, 0, 2, 1, 2, 1, 2, 1, 0, 3, 0, 0, 0, 1, 3, 2, 2, 2, 2, 2, 3, 3, 3, 2, 1, 2, 3, 1, 2, 2, 0, 3, 3, 1, 1, 0, 3, 0, 3, 0, 2, 3, 0, 3, 1, 2, 2, 1, 1, 2, 1, 0, 1, 1, 2, 3, 3, 0, 0, 1, 3, 0, 1, 2, 0, 3, 0, 3, 0, 2, 0, 1, 0, 2, 2, 0, 0, 3, 2, 3, 2, 2, 0, 0, 3, 3, 2, 2, 1, 1, 2, 0, 2, 1, 1, 3, 1]
								}

	map_path='bin/maps/map.html'
	# save the visualized map
	visualize_map(inspection_dict, map_path=map_path)
											
if __name__ == '__main__':
	# # genrate  Nroutes randomly inspected roads 
	# generate_random_roads_ispection()

	# Visuaise a list of inspected roads
	visualize_list_roads_ispection()




