import numpy as np
import cv2
import matplotlib.pyplot as plt
import random

class Node:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = float('inf')  # Used for RRT* rewiring

def load_pgm(filename):
    """Load a PGM file and return the occupancy grid."""
    image = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
    occupancy_grid = np.where(image > 127, 1, 0)  # Convert to binary map (1 = free, 0 = obstacle)
    return occupancy_grid

def distance(n1, n2):
    """Euclidean distance between two nodes."""
    return np.sqrt((n1.x - n2.x) ** 2 + (n1.y - n2.y) ** 2)

def nearest_node(nodes, random_node):
    """Find the nearest node in the tree to the given random node."""
    return min(nodes, key=lambda node: distance(node, random_node))

def steer(from_node, to_node, step_size):
    """Move from 'from_node' toward 'to_node' by 'step_size' while staying collision-free."""
    theta = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
    new_x = int(from_node.x + step_size * np.cos(theta))
    new_y = int(from_node.y + step_size * np.sin(theta))

    if new_x < 0 or new_y < 0 or new_x >= occupancy_grid.shape[1] or new_y >= occupancy_grid.shape[0]:
        return None  # Out of bounds

    if occupancy_grid[new_y, new_x] == 0:
        return None  # Obstacle collision

    return Node(new_x, new_y, from_node)

def collision_free(n1, n2):
    """Check if the line segment between two nodes is collision-free."""
    points = np.linspace((n1.x, n1.y), (n2.x, n2.y), num=20, dtype=int)
    for x, y in points:
        if occupancy_grid[y, x] == 0:  # Obstacle detected
            return False
    return True

def rewire(nodes, new_node, radius):
    """Rewire the tree to find a better path."""
    for node in nodes:
        if distance(node, new_node) < radius and collision_free(node, new_node):
            new_cost = node.cost + distance(node, new_node)
            if new_cost < new_node.cost:
                new_node.parent = node
                new_node.cost = new_cost

def extract_path(goal_node):
    """Extract the final path from goal to start."""
    path = []
    node = goal_node
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]  # Reverse path


occupancy_grid = load_pgm("map.pgm")  # Load your PGM map here
height, width = occupancy_grid.shape

start = Node(190, 200)
#goal = Node(250, 220)
goal = Node(296, 266)

start.cost = 0
nodes = [start]

num_iterations = 5000
step_size = 15
rewire_radius = 15

for _ in range(num_iterations):
    rand_x, rand_y = random.randint(0, width - 1), random.randint(0, height - 1)
    random_node = Node(rand_x, rand_y)
    
    nearest = nearest_node(nodes, random_node)
    new_node = steer(nearest, random_node, step_size)

    if new_node and collision_free(nearest, new_node):
        nodes.append(new_node)
        rewire(nodes, new_node, rewire_radius)

        if distance(new_node, goal) < step_size:
            goal.parent = new_node
            nodes.append(goal)
            print("Goal reached!")
            break

# Extract and visualize the path
path = extract_path(goal)
print(f"Path:{path}")

plt.imshow(occupancy_grid, cmap='gray')
plt.scatter([n.x for n in nodes], [n.y for n in nodes], s=2, c='blue')
plt.plot([x for x, y in path], [y for x, y in path], c='red', linewidth=2)
plt.scatter(start.x, start.y, c='green', marker='o', s=100, label="Start")
plt.scatter(goal.x, goal.y, c='red', marker='x', s=100, label="Goal")
plt.legend()
plt.show()
