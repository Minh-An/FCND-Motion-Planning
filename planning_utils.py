from enum import Enum
from queue import PriorityQueue
from bresenham import bresenham
from scipy.spatial import Voronoi, voronoi_plot_2d
import numpy as np
import numpy.linalg as LA

def create_grid_and_edges(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
    """

    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the min, max coordinates we can compute size of the grid.
    north_size = int(np.ceil((north_max - north_min)))
    east_size = int(np.ceil((east_max - east_min)))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))
    
    # Center offset for grid
    north_min_center = np.min(data[:, 0])
    east_min_center = np.min(data[:, 1])
    
    # Define a list to hold Voronoi points
    points = []
    
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(north - d_north - safety_distance - north_min_center),
                int(north + d_north + safety_distance - north_min_center),
                int(east - d_east - safety_distance - east_min_center),
                int(east + d_east + safety_distance - east_min_center),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1
            
            # add center of obstacles to points list
            points.append([north - north_min, east - east_min])

    # create a voronoi graph based on location of obstacle centers
    graph = Voronoi(points)
    

    edges = []
    for v in graph.ridge_vertices:
        v1 = graph.vertices[v[0]]
        v2 = graph.vertices[v[1]]
        cross = crosses(grid, v1, v2)
        if not cross:
            edge = (tuple(v1), tuple(v2))
            edges.append(edge)
    
    return grid, int(north_min), int(east_min), edges

def crosses(grid, p1, p2):
    if np.amin(p1) < 0 or np.amin(p2)< 0 or p1[0] > grid.shape[0] or p1[1] > grid.shape[1] or p2[0] > grid.shape[0] or p2[1] > grid.shape[1]:
        return True
    
    cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
    for cell in cells:
        #print(cell, p1, p2)
        if grid[cell[0]][cell[1]] == 1:
            return True

    return False

def heuristic(n1, n2):
    return np.linalg.norm(np.array(n2)-np.array(n1))

def a_star(graph, heuristic, start, goal):
    """Modified A* to work with NetworkX graphs."""

    queue = PriorityQueue()
    queue.put((0,start))
    visited = set(start)
    branch = {}

    found = False

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        
        if current_node == start:
            current_cost = 0
        else:
            current_cost = branch[current_node][0]
        
        if current_node == goal:
            found = True
            break
        else:
            for neighbor in graph[current_node]:
                if neighbor not in visited: 
                    g_cost = current_cost + heuristic(neighbor, current_node)
                    f_cost = g_cost + heuristic(neighbor, goal)

                    visited.add(neighbor)
                    queue.put((f_cost, neighbor))
                    branch[neighbor] = (g_cost, current_node)
        
        
    path = []
    path_cost = 0
    if found:
        p = goal
        path_cost = branch[p][0]
        while p != start:
            path.append(p)
            p = branch[p][1]
        path.append(start)
    else:
        print("No path found")
    return path[::-1], path_cost