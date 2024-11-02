import networkx as nx
import osmnx as ox
import math
import numpy as np
from extract import getDir

def loadMap():
    # load map from file
    G = ox.io.load_graphml(filepath=getDir('./fullgraph.osm'))
    print('G: ', G)
    '''
    # impute edge (driving) speeds and calculate edge travel times
    G = ox.speed.add_edge_speeds(G)
    G = ox.speed.add_edge_travel_times(G)

    # you can convert MultiDiGraph to/from GeoPandas GeoDataFrames
    gdf_nodes, gdf_edges = ox.utils_graph.graph_to_gdfs(G)
    G = ox.utils_graph.graph_from_gdfs(gdf_nodes, gdf_edges, graph_attrs=G.graph)

    # convert MultiDiGraph to DiGraph to use nx.betweenness_centrality function
    # choose between parallel edges by minimizing travel_time attribute value
    D = ox.utils_graph.get_digraph(G, weight="travel_time")

    # calculate node betweenness centrality, weighted by travel time
    bc = nx.betweenness_centrality(D, weight="travel_time", normalized=True)
    nx.set_node_attributes(G, values=bc, name="bc")

    # plot the graph, coloring nodes by betweenness centrality
    #nc = ox.plot.get_node_colors_by_attr(G, "bc", cmap="plasma")
    #fig, ax = ox.plot.plot_graph(
    #    G, bgcolor="k", node_color=nc, node_size=50, edge_linewidth=2, edge_color="#333333"
    #)
    '''
    return G
