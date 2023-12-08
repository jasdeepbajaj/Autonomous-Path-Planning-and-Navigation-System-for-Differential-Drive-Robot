import yaml
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import networkx as nx

class PRM_Planner:
    def __init__(self, world_absolute_path: str, start: tuple, end: tuple, num_points: int = 100, min_distance: float = 5.0):
        """
        Initializes the PRM Planner.

        Args:
            world_absolute_path (str): Absolute path to the world YAML file.
            start (tuple): Start coordinates (x, y) as a tuple.
            end (tuple): End coordinates (x, y) as a tuple.
            num_points (int): Number of random points to generate. Default is 100.
            min_distance (float): Minimum distance for connections between points. Default is 5.0.
        """
        world = self.load_world(world_absolute_path)
        mapOG = world['map']
        
        self.grid_array = self.convert_to_occupancy_grid(mapOG)
        
        self.start = start
        self.end = end
        self.num_points = num_points
        self.min_distance = min_distance
        
        self.random_points = self.generate_random_points()
        
        self.connections = self.connect_points()
        self.graph = self.create_graph()
        self.path_coords = self.dijkstra()
        
          
    def load_world(self, file_name:str) -> dict:
        """
        Loads the world information from a YAML file.

        Args:
            file_name (str): Name of the YAML file.

        Returns:
            dict: Loaded world information.
        """
        with open(file_name) as f:
            world = yaml.safe_load(f)   
        return world

    def convert_to_occupancy_grid(self, map_string: str) -> np.array:
        """
        Converts a map string to an occupancy grid.

        Args:
            map_string (str): String representing the map.

        Returns:
            np.array: Occupancy grid as a NumPy array.
        """
        rows = map_string.strip().split('\n')
        occupancy_grid = []

        for row in rows:
            row_values = []
            for char in row:
                if char == '#':
                    row_values.append(1)  # 1 represents occupied space
                elif char == '.':
                    row_values.append(0)  # 0 represents unoccupied space
            occupancy_grid.append(row_values)
            
        occupancy_grid = np.array(occupancy_grid)

        return occupancy_grid
    
    def is_collision(self, point: tuple) -> bool:
        """
        Checks if a given point is in collision with the environment.

        Args:
            point (tuple): Coordinates of the point (x, y) as a tuple.

        Returns:
            bool: True if collision, False otherwise.
        """
        x, y = point
        ext_x_point = int(min(x + 1, self.grid_array.shape[1] - 1)), int(y)
        red_x_point = int(max(x-1, 0)), int(y)
        ext_y_point = int(x), int(min(y + 1, self.grid_array.shape[0]-1))
        red_y_point = int(x), int(max((y-1), 0))

        diag_top_left = int(min(x + 1, self.grid_array.shape[1] - 1)), int(min(y + 1, self.grid_array.shape[0]-1))
        diag_top_right = int(min(x - 1, self.grid_array.shape[1] - 1)), int(min(y + 1, self.grid_array.shape[0]-1))
        diag_bottom_left = int(min(x + 1, self.grid_array.shape[1] - 1)), int(min(y - 1, self.grid_array.shape[0]-1))
        diag_bottom_right = int(min(x - 1, self.grid_array.shape[1] - 1)), int(min(y - 1, self.grid_array.shape[0]-1))

        # if self.grid_array[int(y)][int(x)] == 1 or self.grid_array[ext_x_point[1]][ext_x_point[0]] == 1 or self.grid_array[red_x_point[1]][red_x_point[0]] == 1 or self.grid_array[ext_y_point[1]][ext_y_point[0]] == 1 or self.grid_array[red_y_point[1]][red_y_point[0]] == 1 or self.grid_array[diag_top_left[1]][diag_top_left[0]] == 1 or self.grid_array[diag_top_right[1]][diag_top_right[0]] == 1 or self.grid_array[diag_bottom_left[1]][diag_bottom_left[0]] == 1 or self.grid_array[diag_bottom_right[1]][diag_bottom_right[0]]:
        #     # Assuming 0 represents free space
        #     return True
        # return False
        if any(self.grid_array[int(point[1])][int(point[0])] == 1 for point in [point, ext_x_point, red_x_point, ext_y_point, red_y_point, diag_top_left, diag_top_right, diag_bottom_left, diag_bottom_right]):
            return True
        return False

    
    def generate_random_points(self) -> list:
        """ 
        Generates random points in the environment.

        Returns:
            list: List of generated random points.
        """
        free_points = []
        y_max, x_max = self.grid_array.shape

        while len(free_points) < self.num_points:
            x = np.random.randint(0, x_max)
            y = np.random.randint(0, y_max)
            point = (x, y)
            if not self.is_collision(point):
                free_points.append(point)

        free_points = list(set(free_points))

        free_points.append(self.start)
        free_points.append(self.end)

        return free_points
    
    def connect_points(self) -> list:
        """
        Connects nearby points within a minimum distance.

        Returns:
            list: List of connected points.
        """
        connected_points = []
        kd_tree = KDTree(self.random_points)

        for a, point in enumerate(self.random_points):
            nearby_points = kd_tree.query_ball_point(point, self.min_distance)
            for b in nearby_points:
                if a != b and not self.is_connection_collision(a, b):
                    connected_points.append((a,b))

        return connected_points

    def is_connection_collision(self, a: int, b: int) -> bool:
        """
        Checks if a connection between two points results in a collision with the obstacle

        Args:
            a (int): Index of the first point in random_points list.
            b (int): Index of the second point in random_points list.

        Returns:
            bool: True if connection has collision, False otherwise.
        """
        p1 = self.random_points[a]
        p2 = self.random_points[b]

        p11, p12 = p1[0], p1[1]
        p21, p22 = p2[0], p2[1]

        #result = []
        for i in range(0, 30):
            u = i / 30
            x = p11 * u + p21 * (1 - u)
            y = p12 * u + p22 * (1 - u)

            point = (x, y)

            if self.is_collision(point):
                return True

        return False
    
    def create_graph(self) -> nx.Graph:
        """
        Creates a graph based on connected points.

        Returns:
            networkx.Graph: Graph object representing the connections.
        """
        graph = nx.Graph()
        for connection in self.connections:
            graph.add_edge(connection[0], connection[1])
        return graph

    def dijkstra(self) -> list:
        """
        Performs Dijkstra's algorithm to find the shortest path.

        Returns:
            list: List of coordinates representing the shortest path.
        """
        start_node = np.argmin(np.linalg.norm(np.array(self.random_points) - np.array(self.start), axis=1))
        end_node = np.argmin(np.linalg.norm(np.array(self.random_points) - np.array(self.end), axis=1))

        path = nx.shortest_path(self.graph, start_node, end_node)

        path_coords = [self.random_points[idx] for idx in path]

        return path_coords
    
    def visualize(self):
        """
        Visualizes the occupancy grid, sampled points, connections, and path.
        """
        plt.imshow(self.grid_array, cmap='binary', interpolation='nearest')
        plt.title('Occupancy Grid')
        plt.axis('on')  # Hide axis ticks and labels

        plt.scatter([point[0] for point in self.random_points], [point[1] for point in self.random_points], color='blue', s=30, label='Sampled Points')

        for connection in self.connections:
            point1 = self.random_points[connection[0]]
            point2 = self.random_points[connection[1]]
            plt.plot([point1[0], point2[0]], [point1[1], point2[1]], color='green', alpha=0.5)

        x_coords = [point[0] for point in self.path_coords]
        y_coords = [point[1] for point in self.path_coords]


        plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='red')
        plt.scatter(x_coords, y_coords, color='red')
        plt.show()
