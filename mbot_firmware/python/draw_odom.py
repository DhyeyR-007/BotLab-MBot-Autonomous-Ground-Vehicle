import matplotlib.pyplot as plt

# Initialize empty lists to store (x, y) data
x_data = []
y_data = []

# Read the data from the text file
with open("odometry_data.txt", "r") as file:
    lines = file.readlines()
    for line in lines:
        x, y = map(float, line.split())
        x_data.append(x)
        y_data.append(y)

# Plot the (x, y) data
plt.figure()
plt.plot(x_data, y_data, label='Odometry Path')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Odometry Path')
plt.grid(True)
plt.legend()
plt.show()
