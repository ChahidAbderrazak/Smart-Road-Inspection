import networkx as nx
import osmnx as ox
import numpy as np
import matplotlib.pylab as plt

from lib.utils import *


if __name__ == '__main__':
    # running the CARLA
	Run_carla()

    # Milestone 2 
    push_data_to_firebase()
