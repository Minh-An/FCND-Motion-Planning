import sys
import pkg_resources

# pkg_resources.require("networkx==2.1")
import networkx as nx
import numpy as np
import numpy.linalg as LA
from sklearn.neighbors import KDTree
from shapely.geometry import Polygon, Point
from shapely.geometry import Polygon, Point, LineString
from queue import PriorityQueue
import matplotlib.pyplot as plt

class Poly:

    def __init__(self, coords, height):
        self._polygon = Polygon(coords)
        self._height = height

    @property
    def height(self):
        return self._height

    @property
    def coords(self):
        return list(self._polygon.exterior.coords)[:-1]
    
    @property
    def area(self):
        return self._polygon.area

    @property
    def center(self):
        return (self._polygon.centroid.x, self._polygon.centroid.y)

    def contains(self, point):
        point = Point(point)
        return self._polygon.contains(point)

    def crosses(self, other):
        return self._polygon.crosses(other)

class Map:
    def __init__(self, data, zmax):
        self._data = data
        self._polygons = self.extract_polygons()
        self._xmin = np.min(data[:, 0] - data[:, 3])
        self._xmax = np.max(data[:, 0] + data[:, 3])

        self._ymin = np.min(data[:, 1] - data[:, 4])
        self._ymax = np.max(data[:, 1] + data[:, 4])

        self._zmin = 0
        # limit z-axis
        self._zmax = zmax
        # Record maximum polygon dimension in the xy plane
        # multiply by 2 since given sizes are half widths
        # This is still rather clunky but will allow us to 
        # cut down the number of polygons we compare with by a lot.
        self._max_poly_xy = 2 * np.max((data[:, 3], data[:, 4]))
        centers = np.array([p.center for p in self._polygons])
        self._tree = KDTree(centers, metric='euclidean')
        self._pts = self.sample(300)

    def extract_polygons(self):
        polygons = []

        for i in range(self._data.shape[0]):
            north, east, alt, d_north, d_east, d_alt = self._data[i, :]
            
            obstacle = [north - d_north, north + d_north, east - d_east, east + d_east]
            corners = [(obstacle[0], obstacle[2]), (obstacle[0], obstacle[3]), (obstacle[1], obstacle[3]), (obstacle[1], obstacle[2])]
            
            # TODO: Compute the height of the polygon
            height = alt + d_alt

            p = Poly(corners, height)
            polygons.append(p)

        return polygons

    def sample(self, num_samples):
        """Implemented with a k-d tree for efficiency."""

        xvals = np.random.uniform(self._xmin, self._xmax, num_samples)
        yvals = np.random.uniform(self._ymin, self._ymax, num_samples)
        zvals = np.random.uniform(self._zmin, self._zmax, num_samples)
        samples = list(zip(xvals, yvals, zvals))

        pts = []
        for s in samples:
            in_collision = False
            idxs = list(self._tree.query_radius(np.array([s[0], s[1]]).reshape(1, -1), r=self._max_poly_xy)[0])
            for idx in idxs: 
                p = self._polygons[int(idx)]
                if p.contains(s) and p.height >= s[2]:
                    in_collision = True
            if not in_collision:
                pts.append(s)
                
        return pts

    def can_connect(self, p1, p2):
        ls = LineString([p1, p2])
        for p in self._polygons:
            if p.crosses(ls) and min(p1[2], p2[2]) < p.height:
                return False
        return True

    ## points is list of shapely.geometry.Point(s)
    # k is int param
    def create_graph(self, k):
        g = nx.Graph()
        tree = KDTree(self._pts)
        print(len(self._pts))
        for p in self._pts:
            neighbor_idxs = tree.query([p], k, return_distance=False)[0]
            for i in neighbor_idxs:
                if p !=  self._pts[i] and self.can_connect(p, self._pts[i]):
                    g.add_edge(p, self._pts[i], weight = LA.norm(np.array(p) - np.array(self._pts[i])))
        return g, self._xmin, self._ymin

    @property
    def pts(self):
        return self._pts

def heuristic(self, n1, n2):
    return np.linalg.norm(np.array(n2)-np.array(n1))

def a_star(self, graph, heuristic, start, goal):
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
                g_cost = current_cost + graph[current_node][neighbor]['weight']
                f_cost = g_cost + heuristic(neighbor, goal)

                if neighbor not in visited: 
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