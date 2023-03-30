import math
import heapq
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import KDTree

def distance_point_line(obs, pt1, pt2):
    pt1, pt2 = np.array(pt1), np.array(pt2)
    return np.linalg.norm(np.cross(pt2-pt1, pt1-obs))/np.linalg.norm(pt2-pt1)

class Node:
    def __init__(self, x, y):
        self.point = np.array([x, y])
        self.id = 0

class Edge:
    def __init__(self, start, end):
        self.start = start
        self.end = end
        self.cost = np.linalg.norm(np.array(end) - np.array(start))

class DirectedGraph:
    def __init__(self):
        self.graph = {}
        self.obstacles = []
        self.waypoint_edges = []

    def add_node(self, node):
        if node not in self.graph:
            self.graph[node] = []

    def add_edge(self, start, end):
        if start in self.graph:
            self.graph[start].append(Edge(start, end))
    
    def delete_edge(self, edge):
        self.graph[edge.start].remove(edge)

    def calculate_centroid(self, cities):
        x_sum = sum([city[0] for city in cities])
        y_sum = sum([city[1] for city in cities])
        return (x_sum/len(cities), y_sum/len(cities))
    
    def find_intersecting_edges(self, center, radius):        
        intersecting_edges = []
        for edge in self.waypoint_edges:
            if distance_point_line(center, edge.start, edge.end) <= radius:
                intersecting_edges.append(edge)
        
        return intersecting_edges

    def add_obstacle(self, center, radius, clockwise, num_points=8, fos=1.5):
        self.obstacles.append(center)
        center = np.array(center)
        np_arr_points = np.array([
            center + radius*fos
            * np.array((np.cos(2 * np.pi * i / num_points), np.sin(2 * np.pi * i / num_points)))
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
        
        intersecting_edges = self.find_intersecting_edges(center, radius*fos)
        kdtree = KDTree(np_arr_points)
        for edge in intersecting_edges:
            _, neighbors = kdtree.query(edge.start, k=num_points//2)
            for idx in neighbors:
                self.add_edge(edge.start, points[idx])
                self.waypoint_edges.append(self.graph[edge.start][-1])

            _, neighbors = kdtree.query(edge.end, k=num_points//2)
            for idx in neighbors:
                self.add_edge(points[idx],edge.end)
                self.waypoint_edges.append(self.graph[points[idx]][-1])

            self.delete_edge(edge)
            self.waypoint_edges.remove(edge)

        return
        

    def add_waypoints(self, waypoints):
        for pt in waypoints:
            self.add_node(pt)
        for i in range(1,len(waypoints)):
            self.add_edge(waypoints[i-1], waypoints[i])
            self.waypoint_edges += self.graph[waypoints[i-1]]
        return

    def dijkstra(self, start, end):
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
            for edge in self.graph[current_node]:
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
        return path


    def render(self):
        nodes = list(self.graph.keys())
        edges = [edge for node in nodes for edge in self.graph[node]]
        positions = {node: node for node in nodes}

        fig, ax = plt.subplots()
        for edge in edges:
            ax.annotate("",
                        xy=edge.end,
                        xytext=edge.start,
                        arrowprops=dict(arrowstyle="-|>", connectionstyle="arc3"))

        ax.scatter([p[0] for p in positions.values()], [p[1] for p in positions.values()], s=500, alpha=0.5)
        for node, pos in positions.items():
            ax.text(pos[0], pos[1], node)

        for obs in self.obstacles:
            circle = plt.Circle(obs, 2, color='r')
            ax.add_patch(circle)

        plt.show()



graph = DirectedGraph()
waypoints=[(1,1), (20, 20), (20, 2), (2,20), (1,1)]
graph.add_waypoints(waypoints)
graph.add_obstacle((6,6), 2, True, num_points=6, fos=1.5)
graph.add_obstacle((13,13), 2, True, num_points=6, fos=1.5)

# Sample path between 1,1 and 20,20
print(graph.dijkstra((1,1),(20,20)))

graph.render()
