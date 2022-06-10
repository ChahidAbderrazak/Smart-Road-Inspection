import networkx as nx
import osmnx as ox
import numpy as np
import matplotlib.pylab as plt

ox.config(log_console=True, use_cache=True)
place_coord = (43.94593458427975, -78.89566960105887) # Ontario Tech University
radius = 3000  # meters
G = ox.graph_from_point(place_coord, dist=radius, network_type="drive")


# def plot_road_conditions(G):
available_colors = ['green', 'yellow', 'orange', 'red']
# create Nroutes random routes
Nroutes = 5
route_list = []
route_colors = []
node_colors = []
for iroute in range(Nroutes):
    orig, dest = np.random.choice(G.nodes(), 2)
    route = nx.shortest_path(G, orig, dest, weight='length')
    if len(route) >0:
        route_list.append(route)
        route_color = available_colors[iroute%len(available_colors)]
        route_colors.append(route_color)#[route_color]*(len(route) - 1))

# plot the routes
print(f'\n route_colors = {len(route_colors)}' )
print(f'\n route_list = {len(route_list)}' )
# Plot in map
fig, ax = plt.subplots(figsize=(12,8))
ox.plot_graph_routes( G, route_list, edge_linewidth=1, node_size=0, route_colors = route_colors, ax=ax)
plt.show()
