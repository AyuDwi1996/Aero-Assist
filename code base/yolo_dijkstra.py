# Import necessary libraries
import heapq
import time
from djitellopy import tello
from ultralytics import YOLO
from ultralytics.models.yolo.detect.predict import DetectionPredictor
import cv2

model = YOLO("custom_yolo.pt")
def intializeTello():
    drone = tello.Tello()
    drone.connect()
    drone.for_back_velocity = 0
    drone.left_right_velocity = 0
    drone.up_down_velocity = 0
    drone.yaw_velocity = 0
    drone.speed =0
    print(drone.get_battery())
    drone.streamoff()
    drone.streamon()
    drone.get_frame_read()
    return drone

# Define the GraphNode class for representing nodes in the graph
class GraphNode:
    def __init__(self, name, distance=float('inf')):
        self.name = name
        self.distance = distance
        self.connected_nodes = {}  # Dictionary to store connected nodes and their edge weights

    def add_edge(self, node, direction, distance_weight=1, obstacle_weight=0):
        """
        Add an edge to a neighbor node with direction information.

        Parameters:
            node (GraphNode): The neighbor node to connect to.
            direction (str): The direction of the edge (e.g., 'front', 'back', 'left', 'right').
            distance_weight (int): The weight of the edge representing distance.
            obstacle_weight (int): The weight of the edge representing obstacles.
        """
        self.connected_nodes[node] = {
            'direction': direction,
            'distance_weight': distance_weight,
            'obstacle_weight': obstacle_weight
        }

    def __str__(self):
        return f"Name: {self.name}, Distance: {self.distance}, Connected Nodes: {self.connected_nodes}"

# Function to move the drone based on the given direction
def move_drone(direction, drone):
    if direction == 'back':
        drone.move_backward(20)
        time.sleep(5)  # Adjust the delay as needed
    elif direction == 'left':
        drone.move_left(20)
        time.sleep(5)  # Adjust the delay as needed
    elif direction == 'right':
        drone.move_right(20)
        time.sleep(5)  # Adjust the delay as needed
    elif direction == 'front':
        drone.move_forward(20)
        time.sleep(5)  # Adjust the delay as needed
    # Add other directions as needed
    return


# Function to rotate drone and detect obstacless
def rotate_drone_detect_object(current_vertex, next_vertex, drone):
    obstacle_count = 0

    # Get the direction of the edge from current to next vertex
    edge_info = current_vertex.connected_nodes[next_vertex]
    direction = edge_info['direction']

    while True:
        try:
            # Rotate the drone based on the direction of the edge
            if direction == 'back':
                drone.rotate_counter_clockwise(180)
                time.sleep(5)
            elif direction == 'left':
                drone.rotate_counter_clockwise(90)
                time.sleep(5)
            elif direction == 'right':
                drone.rotate_clockwise(90)
                time.sleep(5)
            elif direction == 'front':
                time.sleep(2)

            # Retrieve the frame
            frame = drone.get_frame_read().frame

            # Perform object detection
            results = model.predict(frame, show=True)
            names = model.names
            detected_objects = [names[int(c)] for r in results for c in r.boxes.cls]

            # Display the frame
            cv2.imshow("Tello Object Detection", frame)

            # Reverse the initial rotation based on the direction
            if direction == 'back':
                drone.rotate_clockwise(180)
                time.sleep(5)
            elif direction == 'left':
                drone.rotate_clockwise(90)
                time.sleep(5)
            elif direction == 'right':
                drone.rotate_counter_clockwise(90)
                time.sleep(5)

            # Check for obstacles in detected objects
            if "stairs" in detected_objects or "pothole" in detected_objects:
                obstacle_count = 1
            break  # Exit the loop after executing the block
        except Exception as e:
            print("Error:", e)
            break  # Exit the loop in case of an error

    return obstacle_count


def dijkstra_shortest_path(graph, source, destination):
    # Initialize distances and predecessors
    distances = {node.name: float('inf') for node in graph}
    predecessors = {node.name: None for node in graph}
    distances[source.name] = 0

    # Priority queue for nodes to visit (distance, node_name)
    priority_queue = [(0, source.name)]
    visited = set()

    while priority_queue:
        # Get the node with the smallest distance from the queue
        current_distance, current_node_name = heapq.heappop(priority_queue)

        # Find the current node
        current_node = next(node for node in graph if node.name == current_node_name)

        # If we have visited this node already, skip it
        if current_node_name in visited:
            continue

        # Mark the current node as visited
        visited.add(current_node_name)

        # If we've reached the destination, stop the search
        if current_node_name == destination.name:
            break

        # Relax the edges
        for neighbor, edge_info in current_node.connected_nodes.items():
            distance_weight = edge_info['distance_weight']

            # Calculate the new distance
            new_distance = current_distance + distance_weight

            # Check if we found a shorter path
            if new_distance < distances[neighbor.name]:
                distances[neighbor.name] = new_distance
                predecessors[neighbor.name] = current_node_name

                # Push the neighbor to the priority queue
                heapq.heappush(priority_queue, (new_distance, neighbor.name))

    # Function to reconstruct the shortest path
    def get_shortest_path(predecessors, source_name, destination_name):
        path = []
        current_node = destination_name

        while current_node:
            path.append(current_node)
            current_node = predecessors[current_node]

        # Reverse the path to get the correct order from source to destination
        path.reverse()
        return path

    # Get the shortest path from the source to the destination
    shortest_path = get_shortest_path(predecessors, source.name, destination.name)

    return shortest_path, distances[destination.name]


def navigate_and_update_path(graph, source, destination, drone):
    # Start at the source node
    current_node = source

    # Repeat the process until reaching the target vertex
    while current_node != destination:
        # Calculate the shortest path from current node to destination
        shortest_path, shortest_distance = dijkstra_shortest_path(graph, current_node, destination)

        # If the shortest path is empty, there is no valid path left
        if not shortest_path:
            print("No valid path found to the destination.")
            return

        # Follow the path derived from Dijkstra's algorithm
        for i in range(len(shortest_path) - 1):
            # Get the current vertex and the next vertex in the path
            current_vertex_name = shortest_path[i]
            next_vertex_name = shortest_path[i + 1]

            # Find the current and next vertices in the graph
            current_vertex = next(node for node in graph if node.name == current_vertex_name)
            next_vertex = next(node for node in graph if node.name == next_vertex_name)

            # Get the edge information from current vertex to next vertex
            edge_info = current_vertex.connected_nodes[next_vertex]
            direction = edge_info['direction']
            distance_weight = edge_info['distance_weight']

            # Stop at the next vertex and test whether the edge includes an obstacle
            obstacle_detected = rotate_drone_detect_object(current_vertex, next_vertex, drone)

            # If an obstacle is found, remove the edge and update the graph
            if obstacle_detected:
                # Remove the edge from current vertex to next vertex
                del current_vertex.connected_nodes[next_vertex]

                # Since the graph has been updated, run Dijkstra's algorithm again
                current_node = current_vertex
                break
            else:
                # Update the current node to the next vertex in the path
                current_node = next_vertex

            # Move the drone in the direction of the edge
            move_drone(direction, drone)


        # If no obstacle was detected and we have reached the destination, stop the loop
        if current_node == destination:
            break

    print(f"Arrived at the destination: {destination.name}")


# Define the graph nodes and edges
start = GraphNode("start", distance=0)
decision1 = GraphNode("decision1", distance=0)
ramp = GraphNode("ramp", distance=0)
stairs = GraphNode("stairs", distance=0)
decision2 = GraphNode("decision2", distance=0)
#pothole = GraphNode("pothole", distance=0)
access_symbol = GraphNode("access_symbol", distance=0)
destination = GraphNode("destination", distance=0)

# Add edges between nodes
start.add_edge(decision1, 'front', distance_weight=1)
decision1.add_edge(access_symbol, 'left', distance_weight=1)
decision1.add_edge(stairs, 'right', distance_weight=1)
access_symbol.add_edge(ramp, 'front', distance_weight=1)
ramp.add_edge(decision2, 'right', distance_weight=1)
stairs.add_edge(destination, 'front', distance_weight=1)
decision2.add_edge(destination, 'right', distance_weight=1)

# Example usage:
# Initialize the drone
drone = intializeTello()
drone.takeoff()
time.sleep(3)
graph = [start, decision1, ramp, stairs, decision2, access_symbol, destination]
source_node = start
destination_node = destination

# Run the navigation and update process
navigate_and_update_path(graph, source_node, destination_node, drone)

