import matplotlib.pyplot as plt
import numpy as np

# Read the file
with open('current.map', 'r') as file:
    metadata = list(map(float, file.readline().split()))
    data = [list(map(float, line.split())) for line in file]

# Convert log odds to probability (if needed)
data = [[1 / (1 + np.exp(-x)) for x in row] for row in data]

# Display the map
plt.imshow(data, cmap='gray', interpolation='none')
plt.show()
