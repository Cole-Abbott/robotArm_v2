# open theta_traj.npy and plot joint angles over time
import numpy as np
import matplotlib.pyplot as plt 

# Load trajectory
theta_traj = np.load("theta_traj.npy")   # shape (N, 6) 

# Plot joint angles
time = np.linspace(0, len(theta_traj)/30, len(theta_traj))  # assuming 30 Hz    
plt.figure()
for i in range(theta_traj.shape[1]):
    plt.plot(time, theta_traj[:, i], label=f'Joint {i+1}')  
plt.xlabel('Time (s)')
plt.ylabel('Joint Angle (rad)')
plt.title('Joint Angles Over Time')
plt.legend()
plt.grid()
plt.show()