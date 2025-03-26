import matplotlib.pyplot as plt

# Function to read velocity data from a text file
def read_velocity_data(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    real_vel_left = []
    goal_vel_left = []

    for line in lines:
        parts = line.strip().split(":")
        if len(parts) == 2:
            type_, values = parts[0], parts[1]
            values = values.strip().strip('[]').split(', ')
            values = [float(v) for v in values]

            if type_ == 'Real (rad/s)':
                real_vel_left.append(values[0])  # Assuming left wheel velocity is the first value
            elif type_ == 'Goal (rad/s)':
                goal_vel_left.append(values[0])  # Assuming left wheel velocity is the first value

    return real_vel_left, goal_vel_left

# Replace with the actual path to your file
file_path = 'mbot_data.txt'
real_vel_left, goal_vel_left = read_velocity_data(file_path)

# Plotting the data
plt.figure(figsize=(12, 6))
plt.plot(real_vel_left, color='blue', label='Real Velocity (left Wheel)')
plt.plot(goal_vel_left, color='red', label='Goal Velocity (left Wheel)')
plt.xlabel('Time Step')
plt.ylabel('Velocity (rad/s)')
plt.title('Real vs Goal Velocity of left Wheel')
plt.legend()
plt.grid(True)

# Save the plot to a file
plt.savefig('left_wheel_velocities_from_file.png')
plt.close()

# Output message
print("Plot saved as 'left_wheel_velocities_from_file.png'")
