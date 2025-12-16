import csv

def generate_square_trajectory(center, side_length, N):
    half_side = side_length / 2
    step = side_length / N
    points = []

    for i in range(N + 1):
        points.append([center[0] - half_side + i * step, center[1] - half_side, center[2]])
    for i in range(1, N + 1):
        points.append([center[0] + half_side, center[1] - half_side + i * step, center[2]])
    for i in range(1, N + 1):
        points.append([center[0] + half_side - i * step, center[1] + half_side, center[2]])
    for i in range(1, N):
        points.append([center[0] - half_side, center[1] + half_side - i * step, center[2]])

    return points

# Define the center, side length, and number of points per side
center = [0.16, 0, 0]
side_length = 0.05
N = 100

# Generate the trajectory
trajectory = generate_square_trajectory(center, side_length, N)

# Write the trajectory to a CSV file
with open('trajectory.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    for point in trajectory:
        writer.writerow(point)



