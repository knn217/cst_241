from load_map import loadMap
import numpy as np
from ford_fulkerson import fordFulkerson
from edmonds_karp import find_maximum_flow_using_edmonds_karp
from dinics import dinics

def getDestList(paths, start, total_travel_time = 0):
    destinations = []
    for i in range(len(paths)):
        if (paths[i]['start'] == start):
            destinations.append({
                'dest': paths[i]['dest'], 
                'travel_time': paths[i]['travel_time'], 
                'total_travel_time': total_travel_time + paths[i]['travel_time'],
                'stop': False 
                })
#    print(destinations)
    return destinations

def bruteForce(paths, start, end, total_travel_time = 0, history = {}):
    #print(history)
    #print(start)
    if (start in history):
        #print('looped')
        return []
    histories = []
    destinations = getDestList(paths, start, total_travel_time)
    #print(np.asarray(destinations))
    for each_dest in destinations:
        new_hist = history.copy()
        new_hist.update({start: total_travel_time})
        if (end in new_hist):
            print('reached dest')
            print(new_hist)
            return [new_hist]    
        histories += bruteForce(paths, each_dest['dest'], end, each_dest['total_travel_time'], new_hist)
        
    return histories


def dijkstra(paths, start, dest):
    # Step 1: Create a graph as an adjacency list
    graph = {}
    for path in paths:
        if path['start'] not in graph:
            graph[path['start']] = []
        graph[path['start']].append((path['dest'], path['travel_time']))

    # Step 2: Initialize data structures
    unvisited_nodes = {node: float('inf') for node in graph}  # Set all nodes' distances to infinity
    unvisited_nodes[start] = 0  # Start node has a distance of zero
    visited_nodes = {}  # Track visited nodes and shortest path times
    previous_nodes = {}  # Track the path taken to each node

    # Step 3: Loop until we visit all reachable nodes or find the destination
    while unvisited_nodes:
        # Select the node with the smallest known distance
        current_node = min(unvisited_nodes, key=unvisited_nodes.get)
        current_distance = unvisited_nodes[current_node]

        # If we reach the destination node, stop and construct the path
        if current_node == dest:
            path = []
            while current_node is not None:
                path.insert(0, current_node)
                current_node = previous_nodes.get(current_node)
            return {'path': path, 'total_time': current_distance}

        # Update the distances to each neighbor of the current node
        for neighbor, travel_time in graph.get(current_node, []):
            if neighbor in visited_nodes:
                continue  # Skip visited neighbors
            new_distance = current_distance + travel_time
            # If a shorter path to the neighbor is found, update the distance and path
            if new_distance < unvisited_nodes.get(neighbor, float('inf')):
                unvisited_nodes[neighbor] = new_distance
                previous_nodes[neighbor] = current_node

        # Mark the current node as visited
        visited_nodes[current_node] = current_distance
        unvisited_nodes.pop(current_node)

    # If we exit the loop without reaching the destination, return no path
    return {'path': [], 'total_time': float('inf')}

def bellman_ford(paths, start, dest):
    # Step 1: Initialize distances from start to all other nodes as infinity
    distances = {}
    previous_nodes = {}
    for path in paths:
        distances[path['start']] = float('inf')
        distances[path['dest']] = float('inf')
    distances[start] = 0

    # Step 2: Relax edges V-1 times
    for _ in range(len(distances) - 1):
        for path in paths:
            u, v, weight = path['start'], path['dest'], path['travel_time']
            if distances[u] != float('inf') and distances[u] + weight < distances[v]:
                distances[v] = distances[u] + weight
                previous_nodes[v] = u

    # Step 3: Reconstruct the shortest path from start to dest
    path = []
    current = dest
    while current in previous_nodes:
        path.insert(0, current)
        current = previous_nodes[current]
    if path:
        path.insert(0, start)

    return {
        'path': path if distances[dest] != float('inf') else [],
        'total_time': distances[dest] if distances[dest] != float('inf') else None
    }

#===============================================================================

import networkx as nx

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
    
    # create capacity for edges
    # for edge in G.edges(data=True):
    #     #print(f'old edge: {edge}')
    #     edge[2]['capacity'] = estimate_max_capacity(edge[2])
    #     #print(f'new edge: {edge}')
        
    max_flow, paths, true_level_graph = fordFulkerson(G, source, sink)
    for path in paths:
        print(path)
    print(true_level_graph)
    print(max_flow)
    #print(level_graph)
