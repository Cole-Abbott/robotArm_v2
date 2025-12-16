import csv
import numpy as np

def generate_line(start, end, N): 
    x = np.linspace(start[0], end[0], N)
    y = np.linspace(start[1], end[1], N)
    z = np.linspace(start[2], end[2], N)
    return np.column_stack((x, y, z))

def generate_lines_trajectory(start, length, num, N):
    z_draw = 0
    z_lift = 0.025 # 6cm

    stepover = 0.02 # 2cm

    points = []

    for i in range(num):
        # lower down
        points.extend(generate_line([start[0], start[1], z_lift], [start[0], start[1], z_draw], N))
        # draw line
        points.extend(generate_line([start[0], start[1], z_draw], [start[0] + length, start[1], z_draw], N))
        # lift up
        points.extend(generate_line([start[0] + length, start[1], z_draw], [start[0] + length, start[1], z_lift], N))
        # move to above next line
        start[1] += stepover
        points.extend(generate_line([start[0] + length, start[1], z_lift], [start[0], start[1], z_lift], N))

    # return to start
    points.extend(generate_line([start[0], start[1], z_lift], [start[0], start[1] - stepover * num, z_lift], N))
        

    return points


# Define the center, side length, and number of points per side
start = [0.16, -0.05, 0]
line_length = 0.05
num = 6
N = 100 # points per line

# Generate the trajectory
trajectory = generate_lines_trajectory(start, line_length, num, N)

# Write the trajectory to a CSV file
with open('../data/trajectory.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    for point in trajectory:
        writer.writerow(point)

