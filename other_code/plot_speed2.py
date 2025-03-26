import matplotlib.pyplot as plt
import numpy as np
# Function to read velocity data from a text file
def read_velocity_data(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()

    real_linear_vel = []
    real_rotational_vel = []
    goal_linear_vel = []
    goal_rotational_vel = []

    # Wheelbase width (adjust with your robot's actual wheelbase width)
    W = 0.077860  # Example width, you need to replace this with the actual wheelbase width of your robot

    for line in lines:
        parts = line.strip().split(":")
        if len(parts) == 2:
            _, values = parts
            values = eval(values.strip())
            left_wheel, right_wheel = values

            linear_vel = (-right_wheel + left_wheel) / 2
            rotational_vel = (right_wheel + left_wheel) / W

            if 'Real' in line:
                real_linear_vel.append(linear_vel)
                real_rotational_vel.append(rotational_vel)
            elif 'Goal' in line:
                goal_linear_vel.append(linear_vel)
                goal_rotational_vel.append(rotational_vel)

    return real_linear_vel, real_rotational_vel, goal_linear_vel, goal_rotational_vel

# Replace with the actual path to your file
file_path = 'drive_square_speed.txt'
real_linear, real_rotational, goal_linear, goal_rotational = read_velocity_data(file_path)

# Plotting the data
plt.figure(figsize=(12, 8))
time_axis = np.linspace(0, 15, num=626)

# Plot real and goal linear velocities
plt.subplot(2, 1, 1)
plt.plot(time_axis, real_linear, label='Real Linear Velocity', color='blue')
plt.plot(time_axis, goal_linear, label='Goal Linear Velocity', color='red')
plt.xlabel('Time Step')
plt.ylabel('Linear Velocity (m/s)')
plt.title('Linear Velocity')
plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), shadow=True, ncol=2)

# Plot real and goal rotational velocities
plt.subplot(2, 1, 2)
plt.plot(time_axis, real_rotational, label='Real Rotational Velocity', color='blue')
plt.plot(time_axis, goal_rotational, label='Goal Rotational Velocity', color='red')
plt.xlabel('Time Step')
plt.ylabel('Rotational Velocity (rad/s)')
plt.title('Rotational Velocity')
plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), shadow=True, ncol=2)

plt.tight_layout()
plt.savefig('222.png')
plt.close()
