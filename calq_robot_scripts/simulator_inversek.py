import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from kinematics import forward_kinematics, inverse_kinematics

# Arm lengths (both 150 mm)
L1 = 112
L2 = 110

# Circle parameters
radius = 78  # Radius of the circle
center_x = 0  # X coordinate of the center of the circle
center_y = L1 + radius - 60  # Y coordinate of the center (we set it vertically above the robot)

# Time and angle parameters
theta = np.linspace(0, 2*np.pi, 100)  # 100 points on the circle

# Circle path
x_path = center_x + radius * np.cos(theta)
y_path = center_y + radius * np.sin(theta)

# Inverse kinematics to calculate joint angles
def inverse_kinematics(x, y):
    # Law of cosines for angle calculation
    d = np.sqrt(x**2 + y**2)
    if d > (L1 + L2):
        raise ValueError("Target point is out of reach.")
    
    # Using law of cosines
    angle2 = np.arccos((d**2 - L1**2 - L2**2) / (2 * L1 * L2))
    angle1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(angle2), L1 + L2 * np.cos(angle2))
    
    return angle1, angle2

# Plot robot animation
fig, ax = plt.subplots()


def plot_robot(angle1, angle2, trajectory_x, trajectory_y):
    # Calculate positions of the joints
    x1 = L1 * np.cos(angle1)
    y1 = L1 * np.sin(angle1)
    x2 = x1 + L2 * np.cos(angle1 + angle2)
    y2 = y1 + L2 * np.sin(angle1 + angle2)
    
    # Clear plot and draw robot arm
    ax.clear()
    ax.plot([0, x1, x2], [0, y1, y2], marker='o', color='b')
    ax.scatter(trajectory_x, trajectory_y, color='r', s=10)  # Plot the trajectory points in red
    ax.set_xlim([-250, 250])  # Set x-axis limits
    ax.set_ylim([-50, 300])  # Set y-axis limits
    ax.set_aspect('equal')
    ax.grid(True)

# Function to update the animation
trajectory_x = []  # To store the x-coordinates of the trajectory
trajectory_y = []  # To store the y-coordinates of the trajectory

def update(frame):
    if frame < len(x_path):  # Ensure we stop after the last frame
        x, y = x_path[frame], y_path[frame]
        angle1, angle2 = inverse_kinematics(x, y)
        
        # Store the trajectory points
        trajectory_x.append(x)
        trajectory_y.append(y)
        
        # Plot the robot and the trajectory
        plot_robot(angle1, angle2, trajectory_x, trajectory_y)

# Create animation with repeat set to False
ani = FuncAnimation(fig, update, frames=len(theta), interval=50, repeat=False)

plt.show()