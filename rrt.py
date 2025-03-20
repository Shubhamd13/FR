import cv2
import numpy as np
import random
import matplotlib.pyplot as plt

# Load the PGM map
map_img = cv2.imread("/home/shubham/map.pgm", cv2.IMREAD_GRAYSCALE)
_, binary_map = cv2.threshold(map_img, 200, 255, cv2.THRESH_BINARY)

height, width = binary_map.shape
occupancy_grid = (binary_map == 255).astype(int)

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = 0  # Cost-to-come

def distance(n1, n2):
    return np.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)

def collision_free(n1, n2):
    points = np.linspace((n1.x, n1.y), (n2.x, n2.y), num=10, dtype=int)
    for x, y in points:
        if occupancy_grid[y, x] == 0:  # Obstacle detected
            return False
    return True

def nearest_node(nodes, rnd):
    return min(nodes, key=lambda node: distance(node, rnd))

def near_nodes(nodes, new_node, radius):
    return [node for node in nodes if distance(node, new_node) < radius]

def steer(from_node, to_node, step_size=10):
    theta = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
    new_x = int(from_node.x + step_size * np.cos(theta))
    new_y = int(from_node.y + step_size * np.sin(theta))

    if new_x >= width or new_y >= height or occupancy_grid[new_y, new_x] == 0:
        return None  # Collision or out of bounds
    return Node(new_x, new_y, from_node)

def rrt_star(start, goal, max_iters=5000, step_size=10, radius=30):
    nodes = [start]
    
    for _ in range(max_iters):
        rand_x = random.randint(0, width - 1)
        rand_y = random.randint(0, height - 1)
        rand_node = Node(rand_x, rand_y)

        nearest = nearest_node(nodes, rand_node)
        new_node = steer(nearest, rand_node, step_size)

        if new_node and occupancy_grid[new_node.y, new_node.x] == 1:
            near = near_nodes(nodes, new_node, radius)
            min_cost = nearest.cost + distance(nearest, new_node)
            best_parent = nearest

            for node in near:
                cost = node.cost + distance(node, new_node)
                if cost < min_cost:
                    min_cost = cost
                    best_parent = node

            new_node.parent = best_parent
            new_node.cost = min_cost
            nodes.append(new_node)

            # Rewiring Step: Check if re-parenting existing nodes reduces cost
            for node in near:
                if node == best_parent:
                    continue
                new_cost = new_node.cost + distance(new_node, node)
                if new_cost < node.cost:
                    node.parent = new_node
                    node.cost = new_cost

            if distance(new_node, goal) < step_size:
                return nodes, new_node
    return None, None

start = Node(160, 140)
goal = Node(250, 220)

nodes, last_node = rrt_star(start, goal)

# Visualize the path
if last_node:
    path = []
    node = last_node
    while node:
        path.append((node.x, node.y))
        node = node.parent

    path = np.array(path)
    plt.imshow(binary_map, cmap='gray')
    plt.plot(path[:, 0], path[:, 1], "-r")
    plt.scatter([start.x, goal.x], [start.y, goal.y], c='blue', marker='o')
    plt.show()
