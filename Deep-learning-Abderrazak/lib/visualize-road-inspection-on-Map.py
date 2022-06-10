#!/usr/bin/env python3
'''Visualize the road conditions map'''
from turtle import color
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import matplotlib.animation as animation
import geopandas as gpd
import osmnx as ox
import folium
ox.settings.log_console = True
ox.__version__

def viz_road_conditions_map(place_coord, radius):
	# get the nearest network nodes to two lat/lng points with the distance module
	start_point = (43.94593458427975, -78.89566960105887)
	dest_point = (43.855273438216805, -78.87272152263739)
	graph = ox.graph_from_point(place_coord, dist=radius)#, network_type="drive")
	origin = ox.distance.get_nearest_node(graph, place_coord)
	
	origin_edge = graph.nodes[origin]
	# print(f'\n - place_coord={place_coord} \n - origin={origin}, \n origin_osmid={origin_edge}')
	start = ox.distance.get_nearest_node(graph, start_point)
	dest = ox.distance.get_nearest_node(graph, dest_point)
	# find the shortest path between nodes, minimizing travel time, then plot it
	route_trajectory = ox.shortest_path(graph, start, dest, weight="travel_time")
	# fig, ax = ox.plot_graph_route(graph, route, node_size=0)

	#get building 
	buildings = ox.geometries_from_point(place_coord, dist=radius, tags={'building':True}) # Retrieve buildings from the area:
	# print(f'\n buildings.columns={buildings.columns}')
	# Retrieve nodes and edges
	nodes, edges = ox.graph_to_gdfs(graph)


	# Retrieve parks
	leisure = ox.geometries_from_point(place_coord, dist=radius, tags={'leisure':True}) # fetch data
	parks = leisure[leisure["leisure"].isin(["pitch","park","playground"])] # select all park polygons
	# Get water bodies map of the New York City area
	water =  ox.geometries_from_point(place_coord, dist=radius, tags={"natural": "water"}) 

	# plot the map
	plot_map(graph, origin_edge, nodes, edges, buildings, parks, water)

def plot_map(graph, origin_edge, nodes, edges, buildings, parks, water):
		# Plot in map
	fig, ax = plt.subplots()#figsize=(12,8))

	# Plot the parks
	parks.plot(ax=ax, facecolor="green")

	# Plot the water
	water.plot(ax=ax, facecolor="blue")

	# Plot buildings
	buildings.plot(ax=ax, facecolor='silver', alpha=0.7)

	# Plot street edges
	edges.plot(ax=ax, linewidth=1, edgecolor='dimgray')
	# print(f'\n - {node} \n\n - {origin_edge}, \n coord_center={coord_center}')
	#- show steets names
	steets_names=[]
	for _, edge in edges.fillna('').iterrows():
		# limit number of street to show
		max_St = 50
		if edge['name'] not in steets_names:
			steets_names.append(edge['name'])
			c = edge['geometry'].centroid
			text = edge['name']
			ax.annotate(text, (c.x, c.y), c='black', fontsize=14, weight='bold')

		if len(steets_names)>max_St:
			break

	# plot route :
	import networkx as nx
	available_colors = ['green', 'yellow', 'orange', 'red']
	# create Nroutes random routes
	Nroutes = 5
	route_list = []
	route_colors = []
	for iroute in range(Nroutes):
			orig, dest = np.random.choice(graph.nodes(), 2)
			route = nx.shortest_path(graph, orig, dest, weight='length')
			if len(route) >0:
					route_list.append(route)
					route_color = available_colors[iroute%len(available_colors)]
					route_colors.append(route_color)#[route_color]*(len(route) - 1))
	# plot the routes
	ox.plot_graph_routes(graph, route_list, edge_linewidth=1, node_size=0, route_colors = route_colors, ax=ax)
	# ax.plot()
	plt.tight_layout()
	plt.show()


if __name__ == '__main__':
	place_coord = (43.94593458427975, -78.89566960105887) # Ontario Tech University
	radius = 3000  # meters
	# vizualise the map
	viz_road_conditions_map(place_coord, radius),
	print(f'\n\n #####  END  #####')


