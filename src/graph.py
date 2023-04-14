import math
import heapq 
import matplotlib  
matplotlib.use('TkAgg')  
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import KDTree
from constants import *

def distance_point_line(obs, pt1, pt2):
    obs = obs[:2]
    pt1, pt2 = np.array(pt1[:2]), np.array(pt2[:2])
    if min(pt1[0],pt2[0])<=obs[0]<=max(pt1[0],pt2[0]) and min(pt1[1],pt2[1])<=obs[1]<=max(pt1[1],pt2[1]):
        return np.linalg.norm(np.cross(pt2-pt1, pt1-obs))/np.linalg.norm(pt2-pt1)
    else:
        return np.inf

class Obstacle:
    def __init__(self, center, is_clockwise):
        self.center = center
        self.clockwise = is_clockwise

class Node:
    def __init__(self, waypoint, edges=[], next_wpt=None):
        self.xy = waypoint[:2]
        self.edges = []
        self.next_wpt = next_wpt

class Edge:
    def __init__(self, start, end):
        self.start = start
        self.end = end
        self.cost = np.linalg.norm(np.array(end) - np.array(start))
        self.vector = np.array(end)-np.array(start)

class DirectedGraph:
    def __init__(self):
        self.graph = {}
        self.obstacles = []
        self.waypoint_edges = []
        self.main_waypoints = []
        self.path = None

    def add_node(self, waypoint, next_wpt=None):
        if waypoint not in self.graph:
            self.graph[waypoint] = Node(waypoint, next_wpt=next_wpt)

    def add_edge(self, start, end):
        if start in self.graph:
            self.graph[start].edges.append(Edge(start, end))
    
    def delete_edge(self, edge):
        self.graph[edge.start].edges.remove(edge)

    def get_next_wpt(self, current_wpt):
        return self.graph[current_wpt].next_wpt

    def calculate_centroid(self, cities):
        """
        Potentitally needed if we dont have the center of the obstacle
        """
        x_sum = sum([city[0] for city in cities])
        y_sum = sum([city[1] for city in cities])
        return (x_sum/len(cities), y_sum/len(cities))
    
    def is_reverse(self, edge, node):
        for other_edge in self.graph[node].edges:
            if np.dot(edge.vector, other_edge.vector) < 0:
                return True
        return False

    def find_intersecting_edges(self, center, radius):   
        """
        Finds trajectory edges that overlap with the obstacles.
        Assumes:
        * Obstacles do not overlap with each other or waypoints
        * Given raidus is premultiplied by FOS
        """     
        intersecting_edges = []
        for edge in self.waypoint_edges:
            if distance_point_line(center, edge.start, edge.end) <= radius:
                intersecting_edges.append(edge)
        
        return intersecting_edges

    def reset(self):
        self.graph = {}
        self.waypoint_edges, self.obstacles = [], []

    def update_obstacles(self, centers, types=None):
        if types is None:
            types = [obs.clockwise for obs in self.obstacles]
        self.reset()
        
        graph.add_waypoints(self.main_waypoints)
        for i in range(len(centers)):
            graph.add_obstacle(centers[i], OBSTACLE_RADIUS, types[i], num_points=NUM_OBSTACLE_POINTS, fos=FOS)
        return

    def add_obstacle(self, center, radius, clockwise, num_points=NUM_OBSTACLE_POINTS, fos=FOS):
        """
        Adds an obstacle to the graph and appropriately wires it to other nodes
        based on whether it lies within the trajectory between two waypoints
        """
        intersecting_edges = self.find_intersecting_edges(center, radius*fos)
        average_z = np.mean([edge.start[-1]+edge.end[-1] for edge in intersecting_edges])/2
        center = center + (average_z,)
        self.obstacles.append(Obstacle(center, clockwise))

        center = np.array(center)
        np_arr_points = np.array([
            center + radius*fos
            * np.array((round(np.cos(2 * np.pi * i / num_points),2), round(np.sin(2 * np.pi * i / num_points), 2), average_z))
            for i in range(num_points)
        ])
        points=tuple(map(tuple, np_arr_points))
        if clockwise:
            sorted_points = sorted(points, key=lambda x: -np.arctan2(x[1]-center[1], x[0]-center[0]))
        else:
            sorted_points = sorted(points, key=lambda x: np.arctan2(x[1]-center[1], x[0]-center[0]))
        
        sorted_points.append(sorted_points[0])
        for pt in sorted_points:
            self.add_node(pt)

        for i in range(1,len(sorted_points)):
            self.add_edge(sorted_points[i-1], sorted_points[i])

        kdtree = KDTree(np_arr_points)
        for edge in intersecting_edges:
            _, neighbors = kdtree.query(edge.start, k=num_points//2)
            for idx in neighbors:
                dists = np.array([distance_point_line(obs.center, edge.start, points[idx]) for obs in self.obstacles[:-1]])
                if not np.any(dists<radius*fos) and not self.is_reverse(Edge(edge.start, points[idx]), points[idx]):
                    # breakpoint()
                    self.add_edge(edge.start, points[idx])
                    self.waypoint_edges.append(self.graph[edge.start].edges[-1])

            _, neighbors = kdtree.query(edge.end, k=num_points//2)
            for idx in neighbors:
                dists = np.array([distance_point_line(obs.center, points[idx], edge.end) for obs in self.obstacles[:-1]])
                if not np.any(dists<radius*fos) and not self.is_reverse(Edge(points[idx], edge.end), points[idx]):
                    self.add_edge(points[idx],edge.end)
                    self.waypoint_edges.append(self.graph[points[idx]].edges[-1])

            self.delete_edge(edge)
            self.waypoint_edges.remove(edge)

        return
        

    def add_waypoints(self, waypoints):
        """
        Adds major waypoints to the graph. To be called at the beginning of the task before
        adding pbstacles.
        """
        waypoints = [tuple(pt) for pt in waypoints]
        self.main_waypoints = waypoints
        for i in range(len(waypoints)-1):
            self.add_node(waypoints[i], next_wpt= waypoints[i+1])
        self.add_node(waypoints[-1])
        for i in range(1,len(waypoints)):
            self.add_edge(waypoints[i-1], waypoints[i])
            self.waypoint_edges += self.graph[waypoints[i-1]].edges
        return

    def dijkstra(self, start, end):
        """
        Written by a certain chatbot.....allegedly
        can change to A* later
        """
        # Initialize the distance and visited dictionaries
        distances = {node: float('inf') for node in self.graph}
        visited = {node: False for node in self.graph}
        predecessor = {node: None for node in self.graph}
        distances[start] = 0
        
        # Initialize the priority queue with the start node and its distance
        pq = [(0, start)]
        
        while pq:
            # Get the node with the smallest distance from the priority queue
            (distance, current_node) = heapq.heappop(pq)
            
            # If we have already visited the node, continue
            if visited[current_node]:
                continue
            
            # Mark the node as visited
            visited[current_node] = True
            
            # Update the distances to the neighboring nodes
            for edge in self.graph[current_node].edges:
                neighbor, weight = edge.end, edge.cost
                new_distance = distances[current_node] + weight
                if new_distance < distances[neighbor]:
                    distances[neighbor] = new_distance
                    predecessor[neighbor] = current_node
                    heapq.heappush(pq, (new_distance, neighbor))
        path = [end]
        current_node = end
        while current_node != start:
            current_node = predecessor[current_node]
            path.append(current_node)
        path.reverse()
        # Return the distance to the end node
        self.path = np.array(path)
        return path


    def modify_path(self, start, end):
        """
        Calls dijkstra and updates the path to next waypoint
        """
        path = self.dijkstra(start, end)
        for i in range(len(path)-1):
            self.graph[path[i]].next_wpt = path[i+1]
        return path

    def print_full_path(self, start):
        curr_pt = start
        while curr_pt is not None:
            print(curr_pt)
            curr_pt = self.get_next_wpt(curr_pt)

    def render(self):
        nodes = list(self.graph.keys())
        edges = [edge for node in nodes for edge in self.graph[node].edges]
        positions = {node: node for node in nodes}

        fig, ax = plt.subplots()
        for edge in edges:
            ax.annotate("",
                        xy=edge.end[:2],
                        xytext=edge.start[:2],
                        arrowprops=dict(arrowstyle="-|>", connectionstyle="arc3"))


        ax.scatter([p[0] for p in positions.values()], [p[1] for p in positions.values()], s=200, alpha=0.5)
        if self.path is not None:
            ax.text(self.path[0][0], self.path[0][1], 'S')
            ax.text(self.path[-1][0], self.path[-1][1], 'G')
            ax.scatter(self.path[:,0], self.path[:,1], s=500, alpha=0.5, color=[0,1,0])

        for obs in self.obstacles:
            circle = plt.Circle(obs.center, OBSTACLE_RADIUS, color='r')
            ax.add_patch(circle)

        plt.show()


# Example
# graph = DirectedGraph()
# waypoints=[(1,1, 2), (25,45, 4), (40, 2, 6), (2,40, 3)]
# obstacles = [(10,10), (60,60), (80,80), (90,90)]
# graph.add_waypoints([np.array(pt) for pt in waypoints])
# graph.update_obstacles(obstacles, INITIAL_OBSTACLE_CLOCKWISE)

# graph.render()
# graph.update_obstacles([(8,10), (25,35), (50,45), (30,10)])
# graph.render()

