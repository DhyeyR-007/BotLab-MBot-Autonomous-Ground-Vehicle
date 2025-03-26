import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from matplotlib.lines import Line2D
import numpy as np
# Function to read data from file
def read_odometry_data(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        data = [eval(line.strip()) for line in lines]
    return data

# Function to plot and save the path
def save_plot(data, filename):
    x_coords = [point[0] for point in data]
    y_coords = [point[1] for point in data]
    thetas = [point[2] for point in data]
    start_theta = thetas[0]
    end_theta = thetas[-1]
    arrow_scale = 0.1

    plt.plot(x_coords, y_coords, linewidth=2)
    # plt.arrow(x_coords[0], y_coords[0], 
    #           arrow_scale * np.cos(start_theta), arrow_scale * np.sin(start_theta),
    #           head_width=0.05, head_length=0.1, fc='green', ec='green', label='Start Orientation')
    
    # # Ending point arrow
    # plt.arrow(x_coords[-1], y_coords[-1], 
    #           arrow_scale * np.cos(end_theta), arrow_scale * np.sin(end_theta),
    #           head_width=0.05, head_length=0.1, fc='red', ec='red', label='End Orientation')
    start_arrow = FancyArrowPatch((x_coords[0], y_coords[0]),
                                (x_coords[0] + arrow_scale * np.cos(start_theta),
                                y_coords[0] + arrow_scale * np.sin(start_theta)),
                                arrowstyle='fancy', color='green', mutation_scale=20, label='Start Orientation')

    # Create arrowhead for the end orientation
    end_arrow = FancyArrowPatch((x_coords[-1], y_coords[-1]),
                                (x_coords[-1] + arrow_scale * np.cos(end_theta),
                                y_coords[-1] + arrow_scale * np.sin(end_theta)),
                                arrowstyle='fancy', color='red', mutation_scale=20, label='End Orientation')

    legend_elements = [Line2D([0], [0], color='blue', lw=2, label='Path'),
                       Line2D([0], [0], marker='>', color='w', markerfacecolor='green', markersize=15, label='Start Orientation'),
                       Line2D([0], [0], marker='>', color='w', markerfacecolor='red', markersize=15, label='End Orientation')]

    # Add patches to the plot
    plt.gca().add_patch(start_arrow)
    plt.gca().add_patch(end_arrow)

    plt.title('Robot Path')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.grid(True)
    plt.legend(handles=legend_elements, loc = 0)
    # Save the plot to a file
    plt.savefig(filename)

# Replace 'your_file.txt' with the path to your text file
data = read_odometry_data('drive_square_4_times.txt')

# Replace 'plot.png' with your desired file name and extension
save_plot(data, 'drive_square_4_times2.png')
