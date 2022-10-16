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

def visualize_list_roads_ispection():
	place_coord = (43.94593458427975, -78.89566960105887) # Ontario Tech University
	radius = 1000  # meters
	max_St = -1
	# create visualization object
	map=HAIS_map(place_coord=place_coord, radius=radius,max_St=max_St)

	list_inspected_coord=[[(44.9506804, -78.8960818), 1],
[(44.950747, -78.8958266), 1],
[(44.950774, -78.89551919999999), 1],
[(44.9507526, -78.89549450000001), 1],
[(44.9507211, -78.8954766), 1],
[(44.9503352, -78.8954126), 1],
[(44.9502605, -78.89540409999999), 1],
[(44.9501825, -78.8953955), 1],
[(44.9500171, -78.8953749), 1],
[(44.9499311, -78.8953637), 1],
[(44.9497545, -78.8953393), 1],
[(44.949576100000004, -78.89531839999999), 1],
[(44.9488408, -78.89522880000001), 1],
[(44.9485513, -78.8951948), 1],
[(44.9476621, -78.89500479999999), 1],
[(44.9475642, -78.8949676), 1],
[(44.9474665, -78.8949261), 1],
[(44.9467789, -78.8946407), 1],
[(44.9466834, -78.8946011), 1],
[(44.946588199999996, -78.8945606), 1],
[(44.946303900000006, -78.8944396), 1],
[(44.946114100000005, -78.894361), 1],
[(44.9460192, -78.894321), 1],
[(44.9459245, -78.8942813), 1],
[(44.9458314, -78.89423950000001), 1],
[(44.94564610000001, -78.8941539), 1],
[(44.94517909999999, -78.8939572), 1],
[(44.945085799999996, -78.8939168), 1],
[(44.9448975, -78.8938369), 1],
[(44.944803599999995, -78.8937995), 1],
[(44.9440532, -78.8934858), 1],
[(44.943959800000004, -78.89344600000001), 1],
[(44.943865999999995, -78.8934064), 1],
[(44.943678900000004, -78.8933263), 1],
[(44.9435855, -78.893287), 1],
[(44.9433071, -78.8931718), 1],
[(44.943033699999996, -78.8930576), 1],
[(44.9427632, -78.892944), 1],
[(44.9426732, -78.8929068), 1],
[(44.9425832, -78.8928694), 1],
[(44.9424933, -78.8928316), 1],
[(44.9421313, -78.8926769), 1],
[(44.941761800000004, -78.8925244), 1],
[(44.941572800000005, -78.8924454), 1],
[(44.9412837, -78.8923221), 1],
[(44.941187, -78.8922793), 1],
[(44.9408964, -78.8921478), 1],
[(44.9407046, -78.8920555), 1],
[(44.9404252, -78.8919005), 1],
[(44.9403327, -78.89184170000001), 1],
[(43.5997915, -78.8915019), 1],
[(43.599704, -78.8914464), 1],
[(43.5993544, -78.8912299), 1],
[(43.5984403, -78.8907741), 1],
[(43.5981632, -78.89065139999999), 1],
[(43.5978843, -78.8905276), 1],
[(43.597791, -78.8904853), 1],
[(43.597033499999995, -78.8901493), 1],
[(43.5966455, -78.5499801), 1],
[(43.5965477, -78.5499373), 1],
[(43.596052799999995, -78.5497203), 1],
[(43.595852400000005, -78.5496328), 1],
[(43.5954468, -78.5494546), 1],
[(43.5949376, -78.549232), 1],
[(43.594837, -78.5491886), 1],
[(43.5946378, -78.5491019), 1],
[(43.5938768, -78.548764), 1],
[(43.5937064, -78.5486889), 1],
[(43.5936227, -78.5486527), 1],
[(43.592891099999996, -78.54832929999999), 1],
[(43.5926614, -78.548229), 1],
[(43.5922906, -78.5480642), 1],
[(43.592216099999995, -78.5480315), 1],
[(43.5920671, -78.5479671), 1],
[(43.591600799999995, -78.5477722), 1],
[(43.591519500000004, -78.5477358), 1],
[(43.5912011, -78.54759659999999), 1],
[(43.5911244, -78.5475641), 1],
[(43.5910475, -78.5475339), 1],
[(43.5908983, -78.5474713), 1],
[(43.5908242, -78.547438), 1],
[(43.5906706, -78.54737519999999), 1],
[(43.5905946, -78.5473379), 1],
[(43.59035050000001, -78.5472272), 1],
[(43.59018269999999, -78.5471471), 1],
[(43.5900997, -78.547107), 1],
[(43.590018300000004, -78.5470705), 1],
[(43.589939, -78.5470356), 1],
[(43.589788799999994, -78.5469668), 1],
[(43.5897176, -78.5469354), 1],
[(43.5896493, -78.5469058), 1],
[(43.5895804, -78.5468752), 1],
[(43.588874100000005, -78.5465744), 1],
[(43.5888079, -78.5465427), 1],
[(43.5887406, -78.5465129), 1],
[(43.5883251, -78.5463353), 1],
[(43.5881099, -78.54624340000001), 1],
[(43.5878248, -78.5461113), 1],
[(43.5874831, -78.545957), 1],
[(43.5874141, -78.5459234), 1],
[(43.5872744, -78.545856), 1],
[(43.5872026, -78.5458227), 1],
[(43.586658, -78.545567), 1],
[(43.586492099999994, -78.5454886), 1],
[(43.5861534, -78.5453351), 1],
[(43.5860675, -78.545295), 1],
[(43.585808500000006, -78.5451764), 1],
[(43.585553600000004, -78.5450621), 1],
[(43.5844667, -78.5446161), 1],
[(43.5842802, -78.54456350000001), 1],
[(43.5838214, -78.5444031), 1],
[(43.583638799999996, -78.5443372), 1],
[(43.5820673, -78.5437029), 1],
[(43.5820033, -78.5436698), 1],
[(43.5818513, -78.5435873), 1],
[(43.5818113, -78.54356539999999), 1],
[(43.5817202, -78.5435156), 1],
[(43.5817172, -78.5435114), 1],
[(43.5816624, -78.5434778), 1],
[(43.5815398, -78.5434089), 1],
[(43.5814303, -78.5433493), 1],
[(43.581371, -78.543318), 1],
[(43.5811793, -78.5432186), 1],
[(43.58110980000001, -78.54318669999999), 1],
[(43.5808223, -78.5430544), 1],
[(43.5807478, -78.5430219), 1],
[(43.5806014, -78.54295619999999), 1],
[(43.580529500000004, -78.5429251), 1],
[(43.5804598, -78.542895), 1],
[(43.580330999999994, -78.5428374), 1],
[(43.580057100000005, -78.5427181), 1],
[(43.5799286, -78.5426646), 1],
[(43.5796944, -78.5425587), 1],
[(43.579531100000004, -78.54249010000001), 1],
[(43.579418, -78.5424416), 1],
[(43.5793626, -78.5424174), 1],
[(43.5793079, -78.54239319999999), 1],
[(43.5790812, -78.5423017), 1],
[(43.578768200000006, -78.5421662), 1],
[(43.5785727, -78.5420957), 1],
[(43.578386300000005, -78.54202790000001), 1],
[(43.57833109999999, -78.5420072), 1],
[(43.5782075, -78.5419565), 1],
[(43.5781348, -78.541925), 1],
[(43.5781238, -78.5419216), 1],
[(43.5781228, -78.5419215), 1],
[(43.5781016, -78.54191259999999), 1],
[(43.578083500000005, -78.5419075), 1],
[(43.578059, -78.5419074), 1],
[(43.578043, -78.5419106), 1],
[(43.578006699999996, -78.5419327), 1],
[(43.5779921, -78.5419573), 1],
[(43.5779421, -78.542114), 1],
[(43.577921399999994, -78.5422102), 1],
[(43.577888599999994, -78.5423564), 1],
[(43.577837099999996, -78.542563), 1],
[(43.577835400000005, -78.5426691), 1],
[(43.5778516, -78.5427442), 1],
[(43.577455099999995, -78.5442101), 1],
[(43.57735219999999, -78.5446075), 1],
[(43.5768674, -78.5454227), 1],
[(43.5766852, -78.5453341), 1],
[(43.575876099999995, -78.5449816), 1],
[(43.5754649, -78.5447516), 1],
[(43.575291, -78.5446613), 1],
[(43.5752478, -78.544641), 1]
												]

	# convert oodinate ro map nodes
	list_inspected_roads=map.convert_coord_to_nodes(list_inspected_coord) 
	input(f'\n flag: list_inspected_roads={list_inspected_roads}')
	# visualize the road conditions
	map.viz_road_conditions(list_inspected_roads)

if __name__ == '__main__':
	# # genrate  Nroutes randomly inspected roads 
	# generate_random_roads_ispection()

	# Visuaise a list of inspected roads
	visualize_list_roads_ispection()




