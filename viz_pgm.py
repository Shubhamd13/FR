import cv2
import matplotlib.pyplot as plt

# Load the PGM file
pgm_file = "map.pgm"  # Replace with your file path
image = cv2.imread(pgm_file, cv2.IMREAD_UNCHANGED)

# Display the image
plt.imshow(image, cmap="gray")
plt.colorbar()
plt.title("PGM Map Visualization")
plt.show()
