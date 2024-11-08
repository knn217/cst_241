from load_map import loadMap

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
def dinics_node(node, level=-1):
    # Add dinics attributes to a node
    #print(f'old: {node}')
    node['level'] = level
    #node.get('level', level)
    #print(f'new: {node}')
    #print(node)
    return node

def dinics_edge(edge, flow=0, capacity=10):
    # Add dinics attributes to a edge
    edge['flow'] = flow
    edge['capacity'] = capacity
    #edge.get('flow', flow)
    #edge.get('capacity', capacity)
    #print(edge)
    return edge

def BFS_buildLevelMap(graph, start_id, end_id):
    '''
    graph: the graph
    start_id: id of start node
    end_id: id of end node
    level_graph: the constructed level graph, return by reference
    '''
    start_node = graph.nodes[start_id]
    end_node = graph.nodes[end_id]
    # Level of source vertex = 0
    dinics_node(start_node, level=0)
    # save nodes in level graph to reset later
    level_graph={'nodes': set(), 'edges': set()}
    level_graph['nodes'].add(start_id)
    
    # Create a queue, enqueue source vertex and mark source vertex as visited
    queue = []
    queue.append(start_id)
    paths = []
    paths.append([start_id])
    true_paths = []
    
    idx = 0
    while queue:
        current_id = queue.pop(0) # pop the 1st id
        #print(f'current id: {current_id}')
        current_node = graph.nodes[current_id]
        path = paths[idx]
        #print(idx, path)
        # get current_node's edges
        for edge in graph.edges(current_id, data=True):
            #print(f'edge: {edge}')
            edge_data = edge[2]
            next_id = edge[1] # get the end node of this edge
            next_node = graph.nodes[next_id]
            #print(f'next node: {next_id}, {next_node}')
            if 'level' not in next_node:
                next_node['level'] = -1
            if 'flow' not in edge_data:
                edge_data['flow'] = 0
            if 'capacity' not in edge_data:
                edge_data['capacity'] = 10
            if next_node['level'] < 0 and edge_data['flow'] < edge_data['capacity']:
                # Level of current vertex is level of parent + 1
                queue.append(next_id)
                level_graph['nodes'].add(next_id)
                level_graph['edges'].add((edge[:2]))
                next_node = dinics_node(next_node, level=current_node['level']+1)
                next_edge = dinics_edge(edge_data)
                #queue.append(dinics_node(next_node, level=current_node['level']+1))
                new_path = path + [next_id]
                paths.append(new_path)
                if next_id == end_id:
                    #print(f'found: {new_path}')
                    true_paths.append(new_path)
                    #print(f'true paths: {true_paths}')
        #print(idx)
        idx+=1
        #if idx == 30:
        #    break
    #print(paths)
    reached_sink = False if ('level' not in end_node or end_node['level'] == -1) else True
    return reached_sink, paths, true_paths, level_graph

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

def DFS_sendFlow(graph, current_id, end_id, flow_in):
    # Sink reached
    if current_id == end_id:
        return flow_in
    total_flow = 0

    current_node = graph.nodes[current_id]    
    # Traverse all adjacent nodes/edges one -by -one
    for edge in graph.edges(current_id, data=True):
        edge_data = edge[2]
        next_id = edge[1] # get the end node of this edge
        next_node = graph.nodes[next_id]
        
        if (next_node['level'] == (current_node['level']+1)) and edge_data['flow'] < edge_data['capacity']:
            # prunes dead ends by ensuring that:
            # only explores edges where the residual capacity is positive.
            # respects the level condition (edges are explored only if the level of the destination node is one greater than the current node's level).
            residual_capacity = edge_data['capacity'] - edge_data['flow']
            if residual_capacity > 0:
                # find minimum flow from u to t
                curr_flow_to_send = min(flow_in, residual_capacity)
                flow_sent = DFS_sendFlow(graph, next_id, end_id, curr_flow_to_send)
                # flow is greater than zero
                if flow_sent and flow_sent > 0:
                    # add flow to current edge
                    edge_data['flow'] += flow_sent
                    # find the reverse edge
                    for rev_edge in graph.edges(next_id, data=True):
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

    
def dinics(graph, start_id, end_id):
    """Find the maximum flow from source to sink"""
    max_flow = 0
    true_paths_list = []
    while True:
        # 1. Build the level graph using BFS
        reached_sink, paths, true_paths, level_graph = BFS_buildLevelMap(graph, start_id, end_id)
        #print(f'old true paths list: {true_paths_list}, {true_paths}')
        true_paths_list += true_paths
        #print(f'new true paths list: {true_paths_list}')
        if not reached_sink:
            break
        # 2. Find augmenting paths using DFS with dead-end pruning
        flow = DFS_sendFlow(graph, start_id, end_id, float('Inf'))
        if flow == 0:
            break
        max_flow += flow
    return max_flow, paths, true_paths_list, level_graph


if __name__ == "__main__":
    #G = loadMap('minigraph.osm')
    G = loadMap('newgraph_conso.osm')
    print(G)
    #for node in G.nodes:
    #    print(f'node: {G.nodes[node]}')
    #    print(f'edge: {G.edges(node, data=True)}')
    #    print(f'edge i: {G.in_edges(node)}')
    #    print(f'edge o: {G.out_edges(node)}')
    
    # 11393762468, 1276696883
    # 11393762468, 11393762467
    #print(G.nodes[533])
    #print(f'edge: {G.out_edges(352, data=True)}') # 1683
    
    
    #BFS_buildLevelMap(G, 11393762468, 1276696883, level_graph)    
    max_flow, paths, true_paths_list, level_graph = dinics(G, 533, 352)
    #print(f'true paths list: {true_paths_list}')
    for path in true_paths_list:
        print(f'paths found: {path}')
        
    print(max_flow)
        
    #print(level_graph)
