import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Arm lengths (both 110 mm)
L1 = 115
L2 = 115

# Circle parameters
radius = 78  # Radius of the circle
center_x = 0  # X coordinate of the center of the circle
center_y = 78  # Y coordinate of the center (we set it vertically above the robot)

# Time and angle parameters for circle path
theta = np.linspace(0, 2 * np.pi, 100)
x_path = center_x + radius * np.cos(theta)
y_path = center_y + radius * np.sin(theta)

# Constraints in radians
theta1_min = np.radians(-20)
theta1_max = np.radians(200)

# Inverse kinematics function
def inverse_kinematics(x, y):
    d = np.sqrt(x**2 + y**2)
    if d > (L1 + L2):
        return None, None  # Target point is out of reach

    angle2 = np.arccos((d**2 - L1**2 - L2**2) / (2 * L1 * L2))
    angle1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(angle2), L1 + L2 * np.cos(angle2))
    
    # Check if angle1 is within constraints
    if angle1 < theta1_min or angle1 > theta1_max:
        return None, None
    
    return angle1, angle2

# Forward kinematics function
def forward_kinematics(angle1, angle2):
    x1 = L1 * np.cos(angle1)
    y1 = L1 * np.sin(angle1)
    x2 = x1 + L2 * np.cos(angle1 + angle2)
    y2 = y1 + L2 * np.sin(angle1 + angle2)
    return x2, y2

# Plotting workspace
def plot_workspace():
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    ax.set_xlim([-250, 250])
    ax.set_ylim([-150, 300])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.grid(True)

    # Generate workspace points
    workspace_x = []
    workspace_y = []
    for angle1 in np.linspace(theta1_min, theta1_max, 200):  # Constrained range for theta1
        for angle2 in np.linspace(-np.pi, np.pi, 200):
            x, y = forward_kinematics(angle1, angle2)
            workspace_x.append(x)
            workspace_y.append(y)

    # Plot the workspace in blue
    ax.scatter(workspace_x, workspace_y, s=1, color='blue', alpha=0.5, label='Reachable Workspace')

    # Plot the circle trajectory in red
    ax.plot(x_path, y_path, color='red', linewidth=2, label='Target Circle Path')

    # Plot robot's end effector path while moving along the circle
    trajectory_x = []
    trajectory_y = []

    for x, y in zip(x_path, y_path):
        angle1, angle2 = inverse_kinematics(x, y)
        if angle1 is not None and angle2 is not None:
            end_x, end_y = forward_kinematics(angle1, angle2)
            trajectory_x.append(end_x)
            trajectory_y.append(end_y)
    
    ax.plot(trajectory_x, trajectory_y, color='green', linestyle='--', label='End Effector Path')
    
    # Add legend
    ax.legend()
    plt.title('2-DOF Robot Arm Workspace and Circle Path')
    plt.show()

# Call the function to plot the workspace and path
plot_workspace()
