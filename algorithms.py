

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
    
def dinics_node(node, flow=0, capacity=0, level=-1, reverse=0):
    # Add dinics attributes to a node
    node.get('flow', flow)
    node.get('capacity', capacity)
    node.get('level', level)
    node.get('reverse', reverse)
    print(node)
    return node

def BFS_buildLevelMap(graph, start_node, end_node, level_graph):
    # Level of source vertex = 0
    dinics_node(start_node, level=0)
    # save nodes in level graph to reset later
    level_graph.append(start_node)
    # Create a queue, enqueue source vertex and mark source vertex as visited
    queue = []
    queue.append(start_node)
    paths = []
    paths.append([start_node])
    print(paths)
    idx = 0
    while queue:
        current_node = queue.pop(0) # pop the 1st element
        path = paths[idx]
        # get its adjacent nodes
        for adj_node in graph.neighbors(current_node):
            if 'level' not in adj_node and 'flow' not in adj_node:
                # Level of current vertex is level of parent + 1
                queue.append(dinics_node(adj_node, level=current_node['level']+1))
                paths.append(path + [adj_node])
        idx+=1
    print(paths)
    # If we can not reach to the sink we return False else True
    return False if ('level' not in end_node or end_node['level'] == -1) else True


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
def DFS_sendFlow(graph, current_node, flow, sink, start):
    # Sink reached
    if current_node == sink:
        return flow
    # Traverse all adjacent nodes/edges one -by -one
    for adj_node in graph.neighbors(current_node):
        if (adj_node['level'] == (current_node['level']+1)) and adj_node['flow'] < adj_node['capacity']:
            # find minimum flow from u to t
            curr_flow = min(flow, adj_node['capacity'] - adj_node['flow'])
            temp_flow = DFS_sendFlow(graph, adj_node, curr_flow, sink, start)
            # flow is greater than zero
            if temp_flow and temp_flow > 0:
                # add flow to current edge
                adj_node['flow'] += temp_flow \#WRONG: key's value needs to be an array for easy search
                # subtract flow from reverse edge of current edge
                self.adj[e.v][e.rev].flow -= temp_flow
                return temp_flow
        
    
def dinics():
    
    return