import os
from lib import utils

def syntax():
    import networkx as nx
    import osmnx as ox
    import numpy as np
    import matplotlib.pylab as plt

    print(f'\n ===>  Sybtax testing')
    ox.config(log_console=True, use_cache=True)
    place_coord = (43.94593458427975, -78.89566960105887) # Ontario Tech University
    radius = 3000  # meters
    G = ox.graph_from_point(place_coord, dist=radius, network_type="drive")
    # Retrieve nodes and edges
    nodes, edges = ox.graph_to_gdfs(G)
    input(nodes)
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

def main_DSP_road_inspection():
    from glob import glob
    # load template-image
    templ_img_path='/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/road-conditions-google/hole/download (2).jpeg'
    road_hole = utils.load_image(templ_img_path)[80:120, 80:120]
    # load image
    img_folder='/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/road-conditions-google/hole/'

    img_folder='/media/abdo2020/DATA1/Datasets/images-dataset/raw-data/road-conditions-google/hole/'
    for img_path in glob(os.path.join(img_folder,'*')):#[1:3]:#
        # DSP-based road inspection
        img, mask = utils.DSP_road_inspection(img_path, road_hole, disp=False)

        img_box, mask_out, nb_box, label_boxes, img_class= utils.create_object_boxes(img, mask, Min_box_area=0, factor=1.2, disp=0)

        utils.display_detection(img, img_box, mask_out, msg='Bouding boxes', cmap="gray")

if __name__ == '__main__':
    # syntax()

    # DSP-based road inspection
    main_DSP_road_inspection()
