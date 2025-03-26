import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the image from file system
image_path = 'draw_maze.png'
image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# Threshold the image to binary
_, binary_map = cv2.threshold(image, 128, 255, cv2.THRESH_BINARY)

# Assuming the starting point is the center of the image
start_point = (350, 350)

# Define the number of rays and the maximum distance they can travel
num_rays = 360
max_distance = max(binary_map.shape)

# Function to cast a ray and find where it hits the wall
def cast_ray(angle, start_point, binary_map, max_distance):
    for r in range(max_distance):
        x = int(start_point[0] + r * np.cos(np.radians(angle)))
        y = int(start_point[1] + r * np.sin(np.radians(angle)))
        # If we are out of bounds or we hit a wall, return the last point
        if x < 0 or y < 0 or x >= binary_map.shape[1] or y >= binary_map.shape[0] or binary_map[y, x] == 0:
            return (x, y)
    return start_point

# Cast all rays
rays_end_points = [cast_ray(angle, start_point, binary_map, max_distance) for angle in range(num_rays)]

# Create a RGB image for visualization
visualized_image = cv2.imread(image_path)

# Draw the rays on the image
for end_point in rays_end_points:
    cv2.line(visualized_image, start_point, end_point, (0, 255, 0), 1)

dot_color = (0, 0, 255) # Red color in BGR
dot_radius = 10 # Size of the dot

# Draw the red dot
cv2.circle(visualized_image, start_point, dot_radius, dot_color, -1)

# Save the resulting image
result_image_path = 'exploration_maze2.png'
cv2.imwrite(result_image_path, visualized_image)

# Display the image with rays
plt.imshow(visualized_image)
plt.show()

# Return the path to the result image
result_image_path
