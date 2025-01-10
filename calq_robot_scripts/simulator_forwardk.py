import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from kinematics import forward_kinematics  # Import the forward kinematics function

# Arm lengths (both 150 mm)
L1 = 120
L2 = 120

# Circle parameters for the trajectory
radius = 78  # Radius of the circle
center_x = 0  # X coordinate of the center of the circle
center_y = L1 + radius - 40  # Y coordinate of the circle's center

# Create circular path in Cartesian coordinates
theta_path = np.linspace(0, 2 * np.pi, 100)  # 100 points for a smooth circle
x_path = center_x + radius * np.cos(theta_path)
y_path = center_y + radius * np.sin(theta_path)

# Initialize the plot
fig, ax = plt.subplots()
ax.set_xlim([-250, 250])  # Set x-axis limits
ax.set_ylim([-50, 300])  # Set y-axis limits
ax.set_aspect('equal')
ax.grid(True)
line, = ax.plot([], [], 'o-', color='b', markersize=10)  # Line for robot arm
end_effector_trace, = ax.plot([], [], 'r.', markersize=5)  # Trace of the end-effector path

# Store the end-effector trajectory for plotting
trajectory_x = []
trajectory_y = []

def init():
    """Initialize the plot background."""
    line.set_data([], [])
    end_effector_trace.set_data([], [])
    return line, end_effector_trace

def inverse_kinematics(x, y):
    """
    Calculate joint angles to reach a given (x, y) position.
    """
    d = np.sqrt(x**2 + y**2)
    if d > (L1 + L2):
        raise ValueError("Target point is out of reach.")
    
    # Using law of cosines
    angle2 = np.arccos((d**2 - L1**2 - L2**2) / (2 * L1 * L2))
    angle1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(angle2), L1 + L2 * np.cos(angle2))
    
    return np.degrees(angle1), np.degrees(angle2)

def update(frame):
    """Update the plot for each frame in the animation."""
    x_target = x_path[frame]
    y_target = y_path[frame]

    # Calculate joint angles using inverse kinematics for the circular trajectory
    theta1, theta2 = inverse_kinematics(x_target, y_target)

    # Calculate forward kinematics for current joint angles to get end-effector position
    x_end, y_end = forward_kinematics(theta1, theta2)
    
    # Calculate positions of the joints
    x1 = L1 * np.cos(np.radians(theta1))
    y1 = L1 * np.sin(np.radians(theta1))
    
    # Update the arm line to show current position
    line.set_data([0, x1, x_end], [0, y1, y_end])
    
    # Store the end-effector position to plot the trajectory
    trajectory_x.append(x_end)
    trajectory_y.append(y_end)
    end_effector_trace.set_data(trajectory_x, trajectory_y)

    return line, end_effector_trace

# Create the animation
ani = FuncAnimation(fig, update, frames=len(theta_path), init_func=init, blit=True, interval=50, repeat=False)

plt.xlabel("X Position (mm)")
plt.ylabel("Y Position (mm)")
plt.title("2-DOF Robot Circular Trajectory Simulation")
plt.show()
