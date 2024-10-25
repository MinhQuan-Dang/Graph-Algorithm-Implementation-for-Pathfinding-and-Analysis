import math
# Function to calculate the Euclidean distance between two vertices
def euclidean_distance(v1, v2):
    """
    v1 and v2 are dictionaries representing coordinates (x, y) of the vertices
    """
    # Use the Euclidean distance formula: sqrt((x2 - x1)^2 + (y2 - y1)^2)
    return math.sqrt((v1['x'] - v2['x'])**2 + (v1['y'] - v2['y'])**2)

# Dijkstra's algorithm to find the shortest path from a start vertex to a goal vertex
def dijkstra(vertices, adjacency_list, start_label, goal_label):
    """
    vertices: dictionary of all vertices with their coordinates
    adjacency_list: adjacency list representing the graph (neighbors and edge weights)
    start_label: the starting vertex
    goal_label: the goal vertex
    """
    # Set of unvisited vertices, initialized with all vertex labels
    unvisited = set(vertices.keys())
    
    # Dictionary to store the current shortest distance from start to each vertex, initialized to infinity
    distances = {vertex: float('inf') for vertex in vertices}
    
    # Dictionary to store the previous vertex in the optimal path for each vertex
    previous = {vertex: None for vertex in vertices}
    
    # Distance to the start vertex is zero, since we are already there
    distances[start_label] = 0

    # Main loop: continue until all vertices are visited
    while unvisited:
        # Select the vertex with the smallest known distance from the unvisited set
        current_vertex = min(
            (vertex for vertex in unvisited), key=lambda vertex: distances[vertex]
        )
        # Remove the selected vertex from the unvisited set
        unvisited.remove(current_vertex)

        # If the smallest distance is infinity, no further reachable vertices remain
        if distances[current_vertex] == float('inf'):
            break

        # Explore each neighbor of the current vertex
        for neighbor, weight in adjacency_list[current_vertex]:
            # Calculate alternative route distance to the neighbor
            alt_route = distances[current_vertex] + weight
            # If the alternative route is shorter, update distance and previous vertex
            if alt_route < distances[neighbor]:
                distances[neighbor] = alt_route
                previous[neighbor] = current_vertex

    # Reconstruct the shortest path by following the previous vertices from goal to start
    path, current = [], goal_label
    while previous[current] is not None:
        path.insert(0, current)  # Insert the current vertex at the beginning of the path
        current = previous[current]  # Move to the previous vertex in the path
    if path:  # If a path was found, add the start vertex at the beginning
        path.insert(0, current)
    
    # Return the reconstructed path and the total distance to the goal vertex
    return path, distances[goal_label]


# Depth-first search (DFS) to find the longest path from start to goal
def dfs_longest_path(adjacency_list, start_label, goal_label, path, visited, max_path, max_distance, current_distance):
    """
    adjacency_list: adjacency list representing the graph (neighbors and edge weights)
    start_label: starting vertex
    goal_label: goal vertex
    path: current path being explored
    visited: set of visited vertices
    max_path: stores the longest path found so far
    max_distance: stores the distance of the longest path found
    current_distance: distance of the current path being explored
    """
    # Mark the current vertex as visited
    visited.add(start_label)
    # Add the current vertex to the current path
    path.append(start_label)

    # If the goal vertex is reached, check if this path is the longest so far
    if start_label == goal_label:
        if current_distance > max_distance[0]:  # Compare current path distance to the longest found
            max_distance[0] = current_distance  # Update longest path distance
            max_path.clear()  # Clear the current longest path
            max_path.extend(path)  # Set the new longest path to the current path
    else:
        # Explore neighbors that have not been visited yet
        for neighbor, weight in adjacency_list[start_label]:
            if neighbor not in visited:
                # Recursively search paths from the neighbor
                dfs_longest_path(
                    adjacency_list, neighbor, goal_label, path, visited, max_path, max_distance, current_distance + weight
                )

    # Backtrack: remove the current vertex from the path and mark it as unvisited
    path.pop()
    visited.remove(start_label)

# MAIN PROGRAM
def main():
    # Read the file name containing the graph from the user
    file_name = input("Enter the file name: ")

    # Read the graph data from the file
    with open(file_name, 'r') as file:
        lines = file.readlines()

    # First line contains the number of vertices and edges
    nVertices, nEdges = map(int, lines[0].split())

    # Parse the vertices with their coordinates from the next nVertices lines
    vertices = {}
    for i in range(1, nVertices + 1):
        label, x, y = lines[i].split()
        vertices[int(label)] = {'x': float(x), 'y': float(y)}

    # Parse the edges and construct the adjacency list (graph)
    adjacency_list = {label: [] for label in vertices}
    for i in range(nVertices + 1, nVertices + nEdges + 1):
        start, end, weight = lines[i].split()
        adjacency_list[int(start)].append((int(end), float(weight)))

    # Last line contains the start and goal vertices
    start_label, goal_label = map(int, lines[-1].split())

    # Output the number of vertices and edges
    print(f"The number of vertexes in the graph: {nVertices}")
    print(f"The number of edges in the graph: {nEdges}")

    # Output the start and goal vertices
    print(f"The start vertexes: {start_label}")
    print(f"The end vertexes: {goal_label}")

    # Calculate and output the Euclidean distance between the start and goal vertices
    start_vertex = vertices[start_label]
    goal_vertex = vertices[goal_label]
    distance = euclidean_distance(start_vertex, goal_vertex)
    print(f"The Euclidean distance between the start and the goal vertex: {distance:.2f}")

    # Find and output the shortest path using Dijkstra's algorithm
    shortest_path, shortest_distance = dijkstra(vertices, adjacency_list, start_label, goal_label)
    print("\nShortest path:")
    if shortest_path:
        print(" -> ".join(map(str, shortest_path)))
        print(f"The length of the shortest path: {shortest_distance}")
    else:
        print("No path found.")

    # Find and output the longest path using DFS
    max_path = []
    max_distance = [float('-inf')]  # Use a list to hold the max distance
    dfs_longest_path(adjacency_list, start_label, goal_label, [], set(), max_path, max_distance, 0)

    print("\nLongest path:")
    if max_path:
        print(" -> ".join(map(str, max_path)))
        print(f"The length of the longest path: {max_distance[0]}")
    else:
        print("No path found.")

if __name__ == "__main__":
    main()
