import numpy as np
import cv2
import matplotlib.pyplot as plt
import random
import yaml

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

def load_map_metadata(yaml_filename):
    """Load YAML map metadata for coordinate conversion."""
    with open(yaml_filename, 'r') as file:
        map_metadata = yaml.safe_load(file)
    
    resolution = map_metadata["resolution"]
    origin_x, origin_y = map_metadata["origin"][:2]  # Extract x, y from origin
    return resolution, origin_x, origin_y

def pixel_to_gazebo(px, py, resolution, origin_x, origin_y, pgm_height):
    """Convert pixel coordinates to Gazebo (real-world) coordinates."""
    gx = origin_x + (px * resolution)
    gy = origin_y + ((pgm_height - py) * resolution)  # Flip Y-axis
    return gx, gy

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


# Load occupancy grid and map metadata
occupancy_grid = load_pgm("inflated_map.pgm")  # Load your PGM map here
height, width = occupancy_grid.shape
resolution, origin_x, origin_y = load_map_metadata("map.yaml")

def world_to_pixel(world_x, world_y,  origin_x,origin_y, resolution, height):
    """Convert Gazebo world coordinates to pixel coordinates."""
    pixel_x = int((world_x - origin_x) / resolution)
    pixel_y = int(height - (world_y - origin_y) / resolution)  # Flip Y-axis
    return pixel_x, pixel_y

# Convert Gazebo coordinates to pixel coordinates
gazebo_x, gazebo_y = 3.0, -2.0
pixel_x, pixel_y = world_to_pixel(gazebo_x, gazebo_y, origin_x,origin_y, resolution, height)
print(f"Start at {pixel_x}, {pixel_y}")
# Define start and goal nodes in pixel coordinates
start = Node(pixel_x, pixel_y)
goal = Node(296, 266)

start.cost = 0
nodes = [start]

# RRT parameters
num_iterations = 5000
step_size = 15
rewire_radius = 15

# Run RRT Algorithm
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

# Extract and convert path to Gazebo coordinates
path_pixels = extract_path(goal)
path_gazebo = [pixel_to_gazebo(px, py, resolution, origin_x, origin_y, height) for px, py in path_pixels]

print(f"Gazebo Waypoints: {path_gazebo}")
print(f"Gazebo Waypoints: {len(path_gazebo)}")


# Visualize the path
plt.imshow(occupancy_grid, cmap='gray')
plt.scatter([n.x for n in nodes], [n.y for n in nodes], s=2, c='blue')
plt.plot([x for x, y in path_pixels], [y for x, y in path_pixels], c='red', linewidth=2)
plt.scatter(start.x, start.y, c='green', marker='o', s=100, label="Start")
plt.scatter(goal.x, goal.y, c='red', marker='x', s=100, label="Goal")
plt.legend()
plt.show()
