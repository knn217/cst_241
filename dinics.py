from collections import deque

# BFS from given source s
def bfs(adj, s):
  
    # Create a queue for BFS
    q = deque()
    
    # Initially mark all the vertices as not visited When we push a node into the q, we mark it as visited
    visited = [False] * len(adj)

    # Mark the source node as visited and enqueue it
    visited[s] = True
    q.append(s)

    # Iterate over the queue
    while q:
      
        # Dequeue a node from queue and print it
        curr = q.popleft()
        print(curr, end=" ")

        # Get all adjacent vertices of the dequeued node. 
        # If an adjacent has not been visited, mark it visited and enqueue it
        for x in adj[curr]:
            if not visited[x]:
                visited[x] = True
                q.append(x)
    
    # queue's view each loop:
    # pop left <- [node_A, node_B,..., node_N]
    # => [node_B..., node_N, node_A_child_1, node_A_child_2,...,node_A_child_M] <- push A's children
    

from collections import deque

# A class to represent the Dinic's algorithm for maximum flow
class Dinic:
    def __init__(self, n):
        self.n = n  # Number of vertices
        self.graph = [[] for _ in range(n)]  # Adjacency list to store the graph
        self.level = [-1] * n  # Level of each node
        self.ptr = [0] * n  # Pointer for DFS

    # Adds a directed edge from u to v with a given capacity
    def add_edge(self, u, v, cap):
        # Forward edge from u to v with capacity 'cap'
        self.graph[u].append([v, cap, len(self.graph[v])])
        # Backward edge from v to u with 0 initial capacity
        self.graph[v].append([u, 0, len(self.graph[u]) - 1])

    # Performs BFS to build the level graph
    def bfs(self, source, sink):
        queue = deque([source])
        self.level = [-1] * self.n
        self.level[source] = 0

        while queue:
            u = queue.popleft()
            for v, cap, rev in self.graph[u]:
                if self.level[v] == -1 and cap > 0:  # Not yet visited and has residual capacity
                    self.level[v] = self.level[u] + 1
                    queue.append(v)

        return self.level[sink] != -1  # If the sink is reachable

    # Performs DFS to send flow along augmenting paths
    def dfs(self, u, sink, flow):
        if u == sink:
            return flow

        while self.ptr[u] < len(self.graph[u]):
            v, cap, rev = self.graph[u][self.ptr[u]]
            if self.level[v] == self.level[u] + 1 and cap > 0:
                # The available flow is the minimum of current flow and the edge's capacity
                pushed = self.dfs(v, sink, min(flow, cap))
                if pushed > 0:
                    # Augment the flow along the path
                    self.graph[u][self.ptr[u]][1] -= pushed  # Reduce capacity in forward edge
                    self.graph[v][rev][1] += pushed  # Increase capacity in backward edge
                    return pushed
            self.ptr[u] += 1

        return 0

    # Main function to calculate the maximum flow
    def max_flow(self, source, sink):
        flow = 0
        while True:
            if not self.bfs(source, sink):  # If there is no augmenting path, we're done
                break
            self.ptr = [0] * self.n  # Reset the pointer for DFS
            while True:
                pushed = self.dfs(source, sink, float('Inf'))
                if pushed == 0:
                    break
                flow += pushed

        return flow

# Driver code
if __name__ == '__main__':
    n, m = map(int, input().split())  # Number of nodes and edges
    dinic = Dinic(n)

    for _ in range(m):
        u, v, cap = map(int, input().split())  # Add edge u -> v with capacity cap
        dinic.add_edge(u, v, cap)

    source, sink = map(int, input().split())  # Source and sink nodes
    print("Maximum Flow:", dinic.max_flow(source, sink))
