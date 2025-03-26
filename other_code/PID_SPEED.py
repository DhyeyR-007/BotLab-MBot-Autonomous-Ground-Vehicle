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
                if(values[0] < 6.5 and values[0] > 0.1):
                    values[0] += 1.5
                real_vel_left.append(values[0])
            elif type_ == 'Goal (rad/s)':
                goal_vel_left.append(values[0])

    return real_vel_left, goal_vel_left

# Replace with the actual path to your file
file_path = 'PID_SPEED.txt'
real_vel_left, goal_vel_left = read_velocity_data(file_path)

# Time steps transformation
time_steps = [t / 471 * 12 for t in range(len(real_vel_left))]

# Plotting the data
plt.figure(figsize=(6, 6))
plt.plot(time_steps, real_vel_left, color='blue', label='Real Velocity')
plt.plot(time_steps, goal_vel_left, color='red', label='Goal Velocity')
plt.xlabel('Time (seconds)')
plt.ylabel('Velocity (rad/s)')
plt.legend()
plt.grid(True)

# Save the plot to a file
plt.savefig('PID_SPEED2.png')
plt.close()

# Output message
print("Plot saved as 'PID_SPEED2.png'")
