def buildResidualGraph(graph):
    residual_graph = {}
    for edge in graph.edges(data=True):
        #print(edge[2])
        if 'capacity' not in edge[2]:
            edge[2]['capacity'] = 10
        u, v, capacity = edge[0], edge[1], edge[2]['capacity']

        # Initialize forward edge
        if u not in residual_graph:
            residual_graph[u] = {}
        if v not in residual_graph[u]:
            residual_graph[u][v] = capacity
        else:
            residual_graph[u][v] += capacity  # Sum capacities if multiple edges

        # Initialize reverse edge with 0 capacity
        if v not in residual_graph:
            residual_graph[v] = {}
        if u not in residual_graph[v]:
            residual_graph[v][u] = 0

    return residual_graph

def dfs(residual_graph, start, dest, visited, path):
    if start == dest:
        return path

    visited.add(start)

    for neighbor, capacity in residual_graph[start].items():
        if neighbor not in visited and capacity > 0:  # Only consider edges with positive capacity
            result = dfs(residual_graph, neighbor, dest, visited, path + [{'start': start, 'end': neighbor, 'capacity': capacity}])
            if result is not None:
                return result
    return None

def fordFulkerson(paths, start, dest):
    residual_graph = buildResidualGraph(paths)
    max_flow = 0
    all_routes = []
    #max_flows = []
    if start not in residual_graph:
        return 0, [], []
    while True:
        # Find an augmenting path using DFS
        visited = set()
        augmenting_path = dfs(residual_graph, start, dest, visited, [])

        if augmenting_path is None:
            break  # No more augmenting paths

        # Calculate the minimum travel time (bottleneck) along the path
        path_flow = min(edge['capacity'] for edge in augmenting_path)

        # Store the path and its minimum travel time
        augmenting_path.append(path_flow)
        all_routes.append(augmenting_path)
        #max_flows.append(path_flow)

        # Augment the flow along the path
        for each_path in augmenting_path[:-1]:
            u, v = each_path['start'], each_path['end']
            residual_graph[u][v] -= path_flow  # Reduce capacity in forward direction
            residual_graph[v][u] += path_flow  # Increase capacity in reverse direction

        max_flow += path_flow  # Increase the total max flow by the path's bottleneck capacity

    return max_flow, all_routes, []

#================================================================================

import networkx as nx
import numpy as np

def create_test_graph():
    G = nx.DiGraph()
    nodes = [
        (1, {'level': -1}),
        (2, {'level': -1}),
        (3, {'level': -1}),
        (4, {'level': -1}),
        (5, {'level': -1}),
        (6, {'level': -1}),
    ]
    edges = [
        (1, 2, {'flow': 0, 'capacity': 10}),
        (1, 3, {'flow': 0, 'capacity': 10}),
        (2, 5, {'flow': 0, 'capacity': 4}),
        (2, 3, {'flow': 0, 'capacity': 2}),
        (3, 4, {'flow': 0, 'capacity': 9}),
        (2, 4, {'flow': 0, 'capacity': 8}),
        (4, 5, {'flow': 0, 'capacity': 6}),
        (5, 6, {'flow': 0, 'capacity': 10}),
        (4, 6, {'flow': 0, 'capacity': 10}),
        (3, 1, {'flow': 0, 'capacity': 0})
    ]
    G.add_nodes_from(nodes)
    G.add_edges_from(edges)
    return G


np.random.seed(10)
def estimate_max_capacity(data):
    max_capacity = 0
    if data['highway']  == 'trunk':
      max_capacity = 500
    elif data['highway']  == ['trunk', 'primary'] or data['highway'] == ['tertiary', 'secondary']:
      max_capacity = 400
    elif data['highway'] == 'primary':
      max_capacity = 300
    elif data['highway'] == 'primary_link' or data['highway'] == 'secondary':
      max_capacity = 200
    elif data['highway'] == 'secondary_link':
      max_capacity = 150
    elif data['highway'] == 'tertiary':
      max_capacity = 100
    elif data['highway'] == 'tertiary_link' or data['highway'] == 'living_street':
      max_capacity = 70
    elif data['highway'] == 'residential':
      max_capacity = 30
    else:
      max_capacity = 20
    return round(max_capacity*(1+np.random.uniform(-0.1, 0.1)))

if __name__ == "__main__":
    #G = loadMap('minigraph.osm')
    #G = loadMap('newgraph_conso.osm')
    G = create_test_graph()
    print(G)
    #for node in G.nodes:
    #    print(f'node: {G.nodes[node]}')
    #    print(f'edge: {G.edges(node, data=True)}')
    #    print(f'edge i: {G.in_edges(node)}')
    #    print(f'edge o: {G.out_edges(node)}')
    
    #source, sink = 533, 352
    source, sink = 1, 6
    
    max_flow, paths, true_level_graph = fordFulkerson(G, source, sink)
    for path in paths:
        print(path)
    print(true_level_graph)
    print(max_flow)
        
