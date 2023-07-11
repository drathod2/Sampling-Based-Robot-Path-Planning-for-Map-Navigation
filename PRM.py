### PRM.py ###

# Standard Algorithm Implementation
# Sampling-based Algorithms PRM

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from scipy import spatial
import math
from numpy import random
from PIL import Image

# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array            # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]    # map size
        self.size_col = map_array.shape[1]    # map size

        self.samples = []                     # list of sampled points
        self.graph = nx.Graph()               # constructed graph
        self.path = []                        # list of nodes of the found path

    def display_map(self, title):
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        pos = {node: (y, x) for node, (x, y) in enumerate(self.samples)}
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])

        # Draw graph nodes and edges
        nx.draw(self.graph, pos, node_size=2, node_color='b', edge_color='y', ax=ax)

        # If a path is found
        if self.path:
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='r')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='r')

        # Draw start and goal nodes
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start', 'goal'], node_size=20, node_color='g')

        # Draw labels for start and goal nodes
        labels = {'start': 'Start', 'goal': 'Goal'}
        node_labels_pos = {node: (pos[node][0], pos[node][1] + 8) for node in labels}
        nx.draw_networkx_labels(self.graph, pos=node_labels_pos, labels=labels, font_size=12, font_color='r')

        
        # Set the title of the plot
        plt.title(title)
        
        # Display the map
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()

    def check_collision(self, p1, p2):
        # Check for obstacles between two nodes
        # Get all the points in between
        points_between = zip(np.linspace(p1[0], p2[0], dtype=int),
                             np.linspace(p1[1], p2[1], dtype=int))
        # Check if any of these points are obstacles
        for p in points_between:
            if self.map_array[p[0]][p[1]] == 0:
                return True
        return False

    def euclidean_distance(self, point1, point2):
        # Calculate the Euclidean distance between two points
        distance = math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
        return distance

    def uniform_sample(self, n_pts):
        # Sample points uniformly on the map
        # Initialize graph
        self.graph.clear()

        points = int(math.sqrt(n_pts))
        xdir = self.size_row
        ydir = self.size_col

        xsample = np.linspace(0, xdir - 1, num=points)  # Equally divide the graph based on number of points
        ysample = np.linspace(0, ydir - 1, num=points)
        xsample = np.round(xsample, decimals=0)  # Get integer values
        ysample = np.round(ysample, decimals=0)

        for i in xsample:
            for j in ysample:
                if self.map_array[int(i), int(j)] != 0:
                    self.samples.append((int(i), int(j)))

    def random_sample(self, n_pts):
        # Sample points randomly on the map
        # Initialize graph
        self.graph.clear()

        points = []
        for i in range(n_pts):
            p1rows = random.randint(self.size_row)  # Randomize points on the map
            p1cols = random.randint(self.size_col)
            p = (p1rows, p1cols)
            points.append(p)

        for i in range(len(points)):
            if self.map_array[points[i][0]][points[i][1]] == 1:  # Check if points are in free space
                p = [points[i][0], points[i][1]]
                self.samples.append(p)

    def gaussian_sample(self, n_pts):
        # Sample points around obstacles using Gaussian distribution
        # Initialize graph
        self.graph.clear()

        self.samples.append((0, 0))  # Start with an initial point
        obs_points = []
        for i in range(n_pts):
            p1rows = random.randint(self.size_row)  # Find random points that are within obstacles
            p1cols = random.randint(self.size_col)
            p = (p1rows, p1cols)
            if self.map_array[p] == 0:
                obs_points.append(p)

        for i in range(len(obs_points)):
            condition = True
            while condition:  # Find points within Gaussian distance and inside obstacles
                p2x = int(np.random.normal(obs_points[i][0], 30))
                p2y = int(np.random.normal(obs_points[i][1], 30))

                if p2x < self.size_row and p2y < self.size_col and self.map_array[(p2x, p2y)] == 1:
                    self.samples.append((p2x, p2y))
                    condition = False

    def bridge_sample(self, n_pts):
        # Sample points between obstacles using Gaussian distribution
        # Initialize graph
        self.graph.clear()
        self.samples.append((0, 0))  # Start with an initial point
        obs_points = []
        for i in range(n_pts):
            p1rows = random.randint(self.size_row)
            p1cols = random.randint(self.size_col)
            p = (p1rows, p1cols)
            if self.map_array[p] == 0:  # Find random points that are within obstacles
                obs_points.append(p)

        for i in range(len(obs_points)):
            p2x = int(np.random.normal(obs_points[i][0], 30))
            p2y = int(np.random.normal(obs_points[i][1], 30))

            if p2x < self.size_row and p2y < self.size_col and self.map_array[(p2x, p2y)] == 0:
                # Find another random point within Gaussian distance and inside another obstacle
                midx = int((obs_points[i][0] + p2x) / 2)
                midy = int((obs_points[i][1] + p2y) / 2)
                if self.map_array[(midx, midy)] == 1:  # If midpoint between the 2 points lies in free space, append it to samples
                    self.samples.append((midx, midy))

    def sample(self, n_pts=1000, sampling_method="uniform"):
        # Generate sample points on the map and construct the graph
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
            connectivity_radius = 15  # Setting connectivity radius between samples
            start_goal_connectivity = 10  # Setting connectivity radius of start/goal with nearby samples

        elif sampling_method == "random":
            self.random_sample(n_pts)
            connectivity_radius = 20
            start_goal_connectivity = 15

        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
            connectivity_radius = 35
            start_goal_connectivity = 75

        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)
            connectivity_radius = 35
            start_goal_connectivity = 75

        pairs = []  # List of pairs of nodes and their distances
        positions = np.array(self.samples)
        kdtree = spatial.KDTree(positions)
        pairsid = kdtree.query_pairs(connectivity_radius)  # Find k nearest neighbors
        pairsid = list(pairsid)

        for i in range(len(pairsid)):
            point11 = self.samples[pairsid[i][0]]  # Take two points from index values
            point21 = self.samples[pairsid[i][1]]
            collision = self.check_collision(point11, point21)
            if collision == True:
                continue
            else:
                distance = self.euclidean_distance(point11, point21)  # Calculate the distance between the points
                if distance != 0:
                    p1index = self.samples.index(point11)
                    p2index = self.samples.index(point21)
                    pairs.append((p1index, p2index, distance))

        self.graph.add_nodes_from([])
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("Constructed graph using %s sampling has %d nodes and %d edges" % (sampling_method.capitalize(), n_nodes, n_edges))
        return start_goal_connectivity

    def search(self, start, goal, start_goal_connectivity, sampling_method):
        # Find a path from start to goal using Dijkstra's algorithm
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)
        # Start and goal nodes will be labeled as 'start' and 'goal' instead of some integer value
        self.graph.add_nodes_from(['start', 'goal'])

        # Connect start and goal to their nearest neighbors within distance 'start_goal_connectivity'
        start_pairs = []
        goal_pairs = []
        for i, point in enumerate(self.samples):
            if point != start:
                collision = self.check_collision(start, point)
                if not collision and self.euclidean_distance(start, point) <= start_goal_connectivity:
                    start_pairs.append(('start', i, self.euclidean_distance(start, point)))
            if point != goal:
                collision = self.check_collision(goal, point)
                if not collision and self.euclidean_distance(goal, point) <= start_goal_connectivity:
                    goal_pairs.append(('goal', i, self.euclidean_distance(goal, point)))

        # Add the edges to the graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)

        # Search using Dijkstra's algorithm
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start', 'goal')
            print("Path found: Length = %.2f" % path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")

        # Draw the resulting path on the map
        self.display_map(sampling_method)

        # Remove start and goal nodes and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)