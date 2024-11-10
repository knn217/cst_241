def set_node(node, level=-1):
    node['level'] = level
    return node

def check_edge(edge, flow=0, capacity=100):
    if 'flow' not in edge:
        edge['flow'] = flow
    if 'capacity' not in edge:
        edge['capacity'] = capacity
    return edge

def cond_1(current_node, next_node, end_node):
    print(next_node)
    if next_node['osmid_original'] == end_node['osmid_original']:
        return True
    
    curr_end = ((current_node['x'] - end_node['x'])**2 + (current_node['y'] - end_node['y'])**2)**0.5
    curr_next = ((current_node['x'] - next_node['x'])**2 + (current_node['y'] - next_node['y'])**2)**0.5
    next_end = ((next_node['x'] - end_node['x'])**2 + (next_node['y'] - end_node['y'])**2)**0.5
    
    cond_1 = curr_end > (4/5)*(curr_next + next_end)
    #cond_2 = curr_end < (curr_next + next_end)
    
    return cond_1# and cond_2 

def BFS_buildLevelMap(graph, start_id, end_id, rev_graph=[], level_graph=None, shortest_dist=None):
    '''
    graph: the graph
    start_id: id of start node
    end_id: id of end node
    level_graph: the constructed level graph, return by reference
    '''
    start_node = graph.nodes[start_id]
    end_node = graph.nodes[end_id]
    # save nodes in level graph to reset later
    if level_graph:
        # reset node levels
        for node_id in level_graph['nodes']:
            node = graph.nodes[node_id]
            node['level'] = -1
        pass
    else:    
        level_graph={'nodes': set(), 'edges': set()}
    level_graph['nodes'].add(start_id)
    # Level of source vertex = 0
    set_node(start_node, level=0)
    
    # Create a queue, enqueue source vertex and mark source vertex as visited
    queue = []
    queue.append(start_id)
    
    while queue:
        current_id = queue.pop(0) # pop the 1st id
        #print(f'current id: {current_id}')
        current_node = graph.nodes[current_id]
        # get current_node's edges
        #print(type(graph.edges(current_id, data=True)))
        for edge in (list(graph.edges(current_id, data=True))):
            rev_edge = list(edge)
            rev_start = rev_edge[1]
            rev_edge[1] = rev_edge[0]
            rev_edge[0] = rev_start
            rev_graph.append(tuple(rev_edge))
            #print(f'edge: {edge}')
            edge_data = edge[2]
            next_id = edge[1] # get the end node of this edge
            next_node = graph.nodes[next_id]
            #print(f'next node: {next_id}, {next_node}')
            
            if 'level' not in next_node:
                next_node['level'] = -1
            check_edge(edge_data)
            
            # condition to put node in level map
            # condition: next node is closer to end node than current node
            if shortest_dist == 'cond_1':
                match = cond_1(current_node, next_node, end_node)
                if not match:
                    print('not match')
                    continue
            if (next_node['level'] == -1) and (edge_data['flow'] < edge_data['capacity']):
                queue.append(next_id)
                # add ids to level graph
                level_graph['nodes'].add(next_id)
                level_graph['edges'].add((edge[:2]))
                # Level of current vertex is level of parent + 1
                set_node(next_node, level=current_node['level']+1)
                check_edge(edge_data)
    reached_sink = False if ('level' not in end_node or end_node['level'] == -1) else True
    return reached_sink, rev_graph, level_graph

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

def DFS_sendFlow(graph, current_id, end_id, rev_graph=[], flow_in=float('Inf'), path=[], paths=[]):
    # Sink reached
    if current_id == end_id:
        path.append(flow_in)
        paths.append(path.copy())
        #print(f'reached end: {path}')
        path.pop()
        return flow_in
    total_flow = 0

    current_node = graph.nodes[current_id]    
    # Traverse all adjacent nodes/edges one -by -one
    for edge in (list(graph.edges(current_id, data=True))):
        edge_data = edge[2]
        next_id = edge[1] # get the end node of this edge
        next_node = graph.nodes[next_id]
        residual_capacity = edge_data['capacity'] - edge_data['flow']
        
        # prunes dead ends by ensuring that:
        # 1. follow the level condition (level of the destination node = current node's level + 1).
        # 2. only explores edges where the residual capacity is positive.
        if (next_node['level'] == (current_node['level']+1)) and residual_capacity > 0:
            # find minimum flow from u to t
            curr_flow_to_send = min(flow_in, residual_capacity)
            path.append({'start': current_id, 'end': next_id, 'flow': flow_in, 'capacity': residual_capacity, 'curr_flow': edge_data['flow']})
            
            flow_sent = DFS_sendFlow(graph, next_id, end_id, rev_graph=rev_graph, flow_in=curr_flow_to_send, path=path, paths=paths)
            
            path.pop()
            
            # only continue if flow is greater than zero
            if not (flow_sent and flow_sent > 0):
                continue
            
            # add flow to current edge
            #print(f'flow sent: {flow_sent}')
            #print(f'old: {edge}')
            edge_data['flow'] += flow_sent
            #print(f'new: {edge}')
            #for e in graph.edges(current_id, data=True):
            #    print(e)
            # find the reverse edge
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
            
            flow_in -= flow_sent
            total_flow += flow_sent
            if flow_in == 0:
                break
    return total_flow

def reset_map(graph, true_level_graph):
    for current_id in true_level_graph['nodes']:
        current_node = graph.nodes[current_id]
        current_node['level'] = -1        
        for edge in graph.edges(current_id, data=True):
            #print(f'edge: {edge}')
            edge_data = edge[2]
            next_id = edge[1] # get the end node of this edge
            next_node = graph.nodes[next_id]
            next_node['level'] = -1
            edge_data['flow'] = 0
    return
    
def dinics(graph, start_id, end_id, shortest_dist=None):
    """Find the maximum flow from source to sink"""
    max_flow = 0
    paths = []
    level_graph = None
    true_level_graph = {'nodes': set(), 'edges': set()}
    flow = 0
    while True:
        # 1. Build the level graph using BFS
        reached_sink, rev_graph, level_graph = BFS_buildLevelMap(graph, start_id, end_id, level_graph=level_graph, shortest_dist=shortest_dist)
        for node_id in level_graph['nodes']:
            node = graph.nodes[node_id]
            #print(node_id, node['level'])
            #print(graph.edges(node_id, data=True))
        
        #print(f'old true paths list: {true_paths_list}, {true_paths}')
        true_level_graph['nodes'] |= level_graph['nodes']
        true_level_graph['edges'] |= level_graph['edges']
        #print(f'new true paths list: {true_paths_list}')
        if not reached_sink:
            break
        # 2. Find augmenting paths using DFS with dead-end pruning
        flow = DFS_sendFlow(graph, start_id, end_id, rev_graph=rev_graph, flow_in=float('Inf'), paths=paths)
        if flow == 0:
            break
        max_flow += flow
    return max_flow, paths, true_level_graph

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
    
    max_flow, paths, true_level_graph = dinics(G, source, sink)
    for path in paths:
        print(path)
    print(true_level_graph)
    print(max_flow)
    reset_map(G, true_level_graph)
    
    print('===============================')
    max_flow, paths, true_level_graph = dinics(G, source, sink)
    for path in paths:
        print(path)
    print(true_level_graph)
    print(max_flow)
    
