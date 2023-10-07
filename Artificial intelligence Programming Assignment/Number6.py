from collections import deque
import heapq

#Weighted graph representation 
weighted_graph = {
    'S': {'A': 3, 'B': 1},
    'A': {'S': 3, 'B': 2, 'C': 2},
    'B': {'B': 1, 'A': 2, 'C': 3},
    'C': {'A': 2, 'B': 3, 'D': 4, 'G': 4},
    'D': {'C': 4, 'G': 1},
    'G': {'C': 4, 'D': 1}
}

#defining the heuristic functions for each node
heuristics = {
    'S': 7,
    'A': 5,
    'B': 7,
    'C': 4,
    'D': 1,
    'G': 0
}

#Breadth-First Search 
def bfs(graph, start, goal):
    visited = set()
    queue = deque([(start, [start])])
    expanded_states = []

    while queue:
        node, path = queue.popleft()
        expanded_states.append(node)
        if node == goal:
            return path, expanded_states, [] # Return an empty list for unexpanded states
        if node not in visited:
            visited.add(node)
            neighbors = sorted(graph[node].keys())
            for neighbor in neighbors:
                if neighbor not in visited:
                    queue.append((neighbor, path + [neighbor]))

    return None, expanded_states, [] # Return an empty list for unexpanded states


#Depth-First Search 
def dfs(graph, start, goal):
    visited = set()
    stack = [(start, [start])]
    expanded_states = []

    while stack:
        node, path = stack.pop()
        expanded_states.append(node)
        if node == goal:
            return path, expanded_states, [] #Return an empty list for unexpanded states
        if node not in visited:
            visited.add(node)
            neighbors = sorted(graph[node].keys())
            for neighbor in reversed(neighbors):
                if neighbor not in visited:
                    stack.append((neighbor, path + [neighbor]))

    return None, expanded_states, [] #Return an empty list for unexpanded states


#Uniform Cost Search 
def ucs(graph, start, goal):
    visited = set()
    priority_queue = [(0, start, [start])]
    expanded_states = []

    while priority_queue:
        cost, node, path = heapq.heappop(priority_queue)
        expanded_states.append(node)
        if node == goal:
            return path, expanded_states, [] #Return an empty list for unexpanded states
        if node not in visited:
            visited.add(node)
            neighbors = sorted(graph[node].keys())
            for neighbor in neighbors:
                if neighbor not in visited:
                    new_cost = cost + graph[node][neighbor]
                    heapq.heappush(priority_queue, (new_cost, neighbor, path + [neighbor]))

    return None, expanded_states, []#Return an empty list for unexpanded states


#A* Search
def astar(graph, start, goal, heuristics):
    visited = set()
    priority_queue = [(heuristics[start], start, [start])]
    expanded_states = []

    while priority_queue:
        if len(priority_queue) == 0:
            break  # No path found
        _, node, path = heapq.heappop(priority_queue)
        expanded_states.append(node)
        if node == goal:
            return path, expanded_states

        if node not in visited:
            visited.add(node)
            neighbors = sorted(graph[node].keys())
            for neighbor in neighbors:
                if neighbor not in visited:
                    cost = path_cost(path) + graph[node][neighbor]
                    heapq.heappush(priority_queue, (cost + heuristics[neighbor], neighbor, path + [neighbor]))

    return None, expanded_states

#Greedy Search
def greedy(graph, start, goal, heuristics):
    visited = set()
    priority_queue = [(heuristics[start], start, [start])]
    expanded_states = []

    while priority_queue:
        _, node, path = heapq.heappop(priority_queue)
        expanded_states.append(node)
        if node == goal:
            return path, expanded_states
        if node not in visited:
            visited.add(node)
            neighbors = sorted(graph[node].keys())
            for neighbor in neighbors:
                if neighbor not in visited:
                    heapq.heappush(priority_queue, (heuristics[neighbor], neighbor, path + [neighbor]))

    unexpanded_states = [node for node in graph if node not in visited]
    return None, expanded_states, unexpanded_states

#Calculate path cost
def path_cost(path):
    cost = 0
    for i in range(len(path) - 1):
        cost += weighted_graph[path[i]][path[i + 1]]
    return cost

#Perform searches
start_node = 'S'
goal_node = 'G'

bfs_result = bfs(weighted_graph, start_node, goal_node)
dfs_result = dfs(weighted_graph, start_node, goal_node)
ucs_result = ucs(weighted_graph, start_node, goal_node)
astar_result = astar(weighted_graph, start_node, goal_node, heuristics)
greedy_result = greedy(weighted_graph, start_node, goal_node, heuristics)


#Breadth-First Search
bfs_path, bfs_expanded, bfs_unexpanded = bfs(weighted_graph, start_node, goal_node)
if bfs_path is not None:
    print("BFS Path:", bfs_path)
else:
    print("BFS did not find a path.")
print("BFS Expanded States:", bfs_expanded)
print("BFS Unexpanded States:", bfs_unexpanded)

#Depth-First Search
dfs_path, dfs_expanded, dfs_unexpanded = dfs(weighted_graph, start_node, goal_node)
if dfs_path is not None:
    print("DFS Path:", dfs_path)
else:
    print("DFS did not find a path.")
print("DFS Expanded States:", dfs_expanded)
print("DFS Unexpanded States:", dfs_unexpanded)

#Uniform Cost Search
ucs_path, ucs_expanded, ucs_unexpanded = ucs(weighted_graph, start_node, goal_node)
if ucs_path is not None:
    print("UCS Path:", ucs_path)
else:
    print("UCS did not find a path.")
print("UCS Expanded States:", ucs_expanded)
print("UCS Unexpanded States:", ucs_unexpanded)


astar_path, astar_expanded = astar(weighted_graph, start_node, goal_node, heuristics)

#Print A* Search results
if astar_path is not None:
    print("A* Search Path:", astar_path)
    print("A* Search Expanded States:", astar_expanded)
else:
    print("A* Search did not find a path.")



greedy_path, greedy_expanded = greedy(weighted_graph, start_node, goal_node, heuristics)

#Print Greedy Search results
if greedy_path is not None:
    print("Greedy Search Path:", greedy_path)
    print("Greedy Search Expanded States:", greedy_expanded)
else:
    print("Greedy Search did not find a path.")




