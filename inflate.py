import cv2
import numpy as np
import yaml
import matplotlib.pyplot as plt

# Load map metadata from YAML
yaml_path = "map.yaml"
with open(yaml_path, 'r') as file:
    map_metadata = yaml.safe_load(file)

resolution = map_metadata["resolution"]  # meters per pixel
origin = map_metadata["origin"][:2]  # (x, y)
map_path = map_metadata["image"]  # Path to .pgm map

# Load map as grayscale image
occupancy_grid = cv2.imread(map_path, cv2.IMREAD_GRAYSCALE)

# Convert to binary (0 = free, 255 = occupied)
binary_map = (occupancy_grid < 127).astype(np.uint8) * 255  

# Robot radius (meters)
robot_radius_m = 0.2  

# Convert robot radius to pixels
inflation_radius_px = int(robot_radius_m / resolution)

# Create circular kernel for dilation
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * inflation_radius_px, 2 * inflation_radius_px))

# Dilate map to inflate obstacles
inflated_map = cv2.dilate(binary_map, kernel, iterations=1)
inflated_map = cv2.bitwise_not(inflated_map)

# Save inflated map
cv2.imwrite("inflated_map.pgm", inflated_map)
plt.imshow(inflated_map, cmap="gray")
plt.colorbar()
plt.title("PGM Map Visualization")
plt.show()
