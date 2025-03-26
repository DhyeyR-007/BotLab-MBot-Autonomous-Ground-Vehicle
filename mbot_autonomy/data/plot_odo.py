import numpy as np
import matplotlib.pyplot as plt

# Load odometry data from file
def load_and_filter_data(filename, threshold=0.01):
    data = np.loadtxt(filename)
    filtered_data = []

    last_x, last_y = None, None
    for x, y, _ in data:
        if last_x is None or abs(x - last_x) > threshold or abs(y - last_y) > threshold:
            filtered_data.append([x, y])
            last_x, last_y = x, y

    return np.array(filtered_data)


# Load and filter the data
odom_data = load_and_filter_data('odom.txt')

# Plotting with thinner lines
plt.plot(odom_data[:, 0], odom_data[:, 1], linewidth=2)  # Adjust linewidth here
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Odometry Data')
plt.savefig('odometry_plot.png')
