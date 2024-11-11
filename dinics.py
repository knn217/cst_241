from collections import deque

def cond_1(residual_graph, curr_id, next_id, end_id):
    curr_x, curr_y = residual_graph[curr_id][curr_id][:2]
    next_x, next_y = residual_graph[next_id][next_id][:2]
    end_x , end_y  = residual_graph[end_id][end_id][:2]
    
    # estimate length with x, y
    curr_end = ((end_x - curr_x)**2 + (end_y - curr_y)**2)**0.5
    next_end = ((end_x - next_x)**2 + (end_y - next_y)**2)**0.5
    curr_next = ((next_x - curr_x)**2 + (next_y - curr_y)**2)**0.5
    
    cond_1 = curr_end > (3)*(curr_next + next_end)
    #cond_2 = curr_end < (curr_next + next_end)
    
    return cond_1# and cond_2 

def cond_2(residual_graph, curr_id, next_id, end_id):
    curr_x, curr_y = residual_graph[curr_id][curr_id][:2]
    next_x, next_y = residual_graph[next_id][next_id][:2]
    end_x , end_y  = residual_graph[end_id][end_id][:2]
    
    # path length since this is the only path
    curr_next = residual_graph[curr_id][next_id][2]
    
    # estimate length with x, y
    curr_end = ((end_x - curr_x)**2 + (end_y - curr_y)**2)**0.5
    next_end = ((end_x - next_x)**2 + (end_y - next_y)**2)**0.5
    
    cond_1 = curr_end > (2/3)*(curr_next + next_end)
    #cond_2 = curr_end < (curr_next + next_end)
    
    return cond_1# and cond_2 

def get_residual_graph(graph):
        residual = {}
        for u, v, _data in graph.edges(data=True):
            if u not in residual:
                residual[u] = {}
                # Add level, x, y
                node_u = graph.nodes[u]
                residual[u][u] = [node_u.get('x', 0), node_u.get('y', 0), -1]
            if v not in residual:
                residual[v] = {}
                # Add level, x, y
                node_v = graph.nodes[v]
                residual[v][v] = [node_v.get('x', 0), node_v.get('y', 0), -1]
            
            # Add forward edge [flow, capacity, length]
            if v not in residual[u]:
                residual[u][v] = [0, 0, 0]
            
            # Add capacity and length but avoid levels
            if u != v:
                residual[u][v][1] += _data.get('capacity', 0)
                residual[u][v][2] += _data.get('length', 0)
            
            # Add reverse edge if it doesn't exist
            if u not in residual[v]:
                residual[v][u] = [0, 0, 0]
                
        return residual

def BFS_buildLevelMap(residual_graph, start_id, end_id, shortest_dist=None):
    '''
    graph: the graph
    start_id: id of start node
    end_id: id of end node
    level_graph: the constructed level graph, return by reference
    '''
    # reset node levels -> -1, except start node level -> 0
    #print(residual_graph)
    for u in residual_graph:
        #print(f'node_id: {u}')
        residual_graph[u][u][-1] = 0 if u ==start_id else -1
    
    # Create a queue, enqueue source vertex and mark source vertex as visited
    queue = deque([start_id])
    
    while queue:
        u = queue.popleft() # pop the 1st id
        level_u = residual_graph[u][u][-1] # current node's level
        for v in residual_graph[u]:
            if v == u:
                continue
            flow_e, cap_e, len_e = residual_graph[u][v] # edge data: flow, capacity
            level_v = residual_graph[v][v][-1] # next node's level
            
            # condition to put node in level map
            # condition: next node is closer to end node than current node
            if shortest_dist == 'cond_1' and v != end_id:
                match = cond_1(residual_graph, u, v, end_id)
                if not match:
                    #print('not match')
                    continue
            if (level_v == -1) and (flow_e < cap_e): # next node's level is not set
                # add v to queue
                queue.append(v)
                # update next node's level
                residual_graph[v][v][-1] = level_u + 1
                
    reached_sink = False if (residual_graph[end_id][end_id] == -1) else True
    return reached_sink

# A DFS based function to send flow after BFS has
# figured out that there is a possible flow and
# constructed levels. This functions called multiple
# times for a single call of BFS.
# 
# flow : Current flow send by parent function call
# start[] : To keep track of next edge to be explored
#           start[i] stores count of edges explored
#           from i
# u : Current vertex
# t : Sink

def DFS_sendFlow(residual_graph, u, end_id, flow_in=float('Inf'), path=[], paths=[]):
    # Sink reached
    if u == end_id:
        path.append(flow_in)
        paths.append(path.copy())
        #print(f'reached end: {path}')
        path.pop()
        return flow_in
    total_flow = 0

    level_u = residual_graph[u][u][-1] # current node's level
    # Traverse all adjacent nodes/edges one -by -one
    for v in residual_graph[u]:
        if v == u:
            continue
        flow_e, cap_e, len_e = residual_graph[u][v] # edge data: flow, capacity
        level_v = residual_graph[v][v][-1] # next node's level
        residual_cap = cap_e - flow_e
        
        # prunes dead ends by ensuring that:
        # 1. follow the level condition (level of the destination node = current node's level + 1).
        # 2. only explores edges where the residual capacity is positive.
        if (level_v == (level_u+1)) and residual_cap > 0:
            # find minimum flow from u to t
            curr_flow_to_send = min(flow_in, residual_cap)
            #path.append({'start': current_id, 'end': next_id, 'flow': flow_in, 'capacity': residual_capacity, 'curr_flow': edge_data['flow']})
            traveled = path[-1].get('traveled', 0) if path else 0
            path.append({'start': u, 'end': v, 'capacity': residual_cap, 'length': len_e, 'traveled': len_e + traveled})
            
            flow_sent = DFS_sendFlow(residual_graph, v, end_id, flow_in=curr_flow_to_send, path=path, paths=paths)
            
            path.pop()
            
            # only continue if flow is greater than zero
            if not (flow_sent and flow_sent > 0):
                continue
            
            # add flow to current edge
            #print(f'flow sent: {flow_sent}')
            #print(f'old: {edge}')
            residual_graph[u][v][0] += flow_sent
            residual_graph[v][u][0] -= flow_sent
            #print(f'new: {edge}')
            #for e in graph.edges(current_id, data=True):
            #    print(e)
            # find the reverse edge
            '''
            for rev_edge in list(graph.edges(next_id, data=True)):
            #for rev_edge in rev_graph:
                if rev_edge[1] == current_id:
                    rev_edge_data = rev_edge[2]
                    # subtract flow from reverse edge of current edge
                    if 'flow' not in rev_edge_data:
                        rev_edge_data['flow'] = -flow_sent
                        break
                    rev_edge_data['flow'] -= flow_sent
                    break
            '''
            flow_in -= flow_sent
            total_flow += flow_sent
            if flow_in == 0:
                break
    return total_flow

def dinics(graph, start_id, end_id, shortest_dist=None):
    """Find the maximum flow from source to sink"""
    residual_graph = get_residual_graph(graph)
    max_flow = 0
    paths = []
    flow = 0
    while True:
        # 1. Build the level graph using BFS
        reached_sink = BFS_buildLevelMap(residual_graph, start_id, end_id, shortest_dist=shortest_dist)
        #print(residual_graph)
        
        if not reached_sink:
            break
        # 2. Find augmenting paths using DFS with dead-end pruning
        flow = DFS_sendFlow(residual_graph, start_id, end_id, flow_in=float('Inf'), paths=paths)
        if flow == 0:
            break
        max_flow += flow
    return max_flow, paths

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
    
    max_flow, paths = dinics(G, source, sink)
    for path in paths:
        print(path)
    print(max_flow)
    
    print('===============================')
    max_flow, paths = dinics(G, source, sink)
    for path in paths:
        print(path)
    print(max_flow)
    
