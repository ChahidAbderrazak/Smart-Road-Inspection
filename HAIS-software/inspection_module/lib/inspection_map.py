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

class HAIS_visualization(object):
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
		#get building 
		self.buildings = ox.geometries_from_point(self.place_coord, dist=self.radius, tags={'building':True}) # Retrieve buildings from the area:
		# print(f'\n buildings.columns={buildings.columns}')
		# Retrieve parks
		self.leisure = ox.geometries_from_point(self.place_coord, dist=self.radius, tags={'leisure':True}) # fetch data
		self.parks = self.leisure[self.leisure["leisure"].isin(["pitch","park","playground"])] # select all park polygons
		# Get water bodies map of the New York City area
		self.water =  ox.geometries_from_point(self.place_coord, dist=self.radius, tags={"natural": "water"}) 

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
			# get the earest node to the locations
			start = ox.nearest_nodes(self.graph, X=start_point[0],Y=start_point[1])
			dest = ox.nearest_nodes(self.graph, X=dest_point[0],Y=dest_point[1])
			if start!=dest:
				# get random road conditions
				road_metric = random.randint(0, 4)
				list_coodinate.append((start, dest, road_metric))
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
		


if __name__ == '__main__':
	place_coord = (43.94593458427975, -78.89566960105887) # Ontario Tech University
	radius = 1000  # meters
	max_St = -1
	# create visualization object
	map=HAIS_visualization(place_coord=place_coord, radius=radius,max_St=max_St)

	# genrate  Nroutes randomly inspected roads 
	Nroutes=30
	list_inspected_roads = map.generate_random_road_slices(Nroutes=Nroutes)
	# visualize the road conditions
	map.viz_road_conditions(list_inspected_roads)
	print(f'\n\n #####  END  #####')


