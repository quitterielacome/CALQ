import numpy as np
import matplotlib.pyplot as plt

# Parameters for robot arm
r1 = 120  # Length of first link
r2 = 112  # Length of second link
t1 = 0  # Initial angle of joint 1
t2 = 0  # Initial angle of joint 2

# Desired position
x_d = np.array([0, 215])  # Desired x and y coordinates of the end-effector

# Control parameters
damping_factor = 0.01  # Damping factor for DLS
step_limit = 0.1  # Maximum step size for target clamping
gain = 0.3  # Gain for Jacobian transpose method
tolerance = 1e-3  # Position error tolerance
max_iterations = 1000  # Maximum number of iterations
dt = 0.01  # Time step for plotting manipulability
ellipsoid_scale = 0.25  # Scaling factor for ellipsoids

# Function to calculate forward kinematics
def forward_kinematics(r1, r2, t1, t2):
    x = r1 * np.cos(t1) + r2 * np.cos(t1 + t2)
    y = r1 * np.sin(t1) + r2 * np.sin(t1 + t2)
    return np.array([x, y])

# Function to compute the Jacobian
def compute_jacobian(r1, r2, t1, t2):
    J = np.array([
        [-r1 * np.sin(t1) - r2 * np.sin(t1 + t2), -r2 * np.sin(t1 + t2)],
        [r1 * np.cos(t1) + r2 * np.cos(t1 + t2),  r2 * np.cos(t1 + t2)]
    ])
    return J

# Initialize array to store manipulability values
manipulability_values = []

# Set up the plot
plt.figure(1)
plt.title('Robot Arm Motion and Manipulability Ellipsoids')
plt.xlabel('X')
plt.ylabel('Y')
plt.axis('equal')
plt.xlim([-2, 2])
plt.ylim([-2, 2])
plt.grid(True)

for i in range(max_iterations):
    # Calculate forward kinematics
    current_position = forward_kinematics(r1, r2, t1, t2)
    
    # Calculate position error
    error = x_d - current_position
    
    # Target clamping: Limit the step size
    if np.linalg.norm(error) > step_limit:
        error = (error / np.linalg.norm(error)) * step_limit
    
    # Compute the Jacobian
    J = compute_jacobian(r1, r2, t1, t2)
    
    # Calculate manipulability
    manipulability = np.sqrt(np.linalg.det(J @ J.T))
    manipulability_values.append(manipulability)
    
    # Damped Least Squares (DLS) method
    inv_J_damped = np.linalg.pinv(J.T @ J + damping_factor * np.eye(2)) @ J.T
    delta_q_dls = inv_J_damped @ error
    
    # Jacobian Transpose method
    delta_q_transpose = gain * J.T @ error
    
    # Select method
    delta_q = delta_q_transpose  # Uncomment this to use Jacobian Transpose
    # delta_q = delta_q_dls      # Uncomment this to use DLS
    
    # Update joint angles
    t1 += delta_q[0]
    t2 += delta_q[1]
    
    # Plot current robot arm position
    x1 = r1 * np.cos(t1)
    y1 = r1 * np.sin(t1)
    x2 = x1 + r2 * np.cos(t1 + t2)
    y2 = y1 + r2 * np.sin(t1 + t2)
    plt.plot([0, x1, x2], [0, y1, y2], '-o', color='blue')
    
    # Plot the manipulability ellipsoid
    U, S, Vt = np.linalg.svd(J @ J.T)
    theta = np.linspace(0, 2 * np.pi, 100)
    ellipse_x = ellipsoid_scale * np.sqrt(S[0]) * np.cos(theta)
    ellipse_y = ellipsoid_scale * np.sqrt(S[1]) * np.sin(theta)
    ellipse = np.array([ellipse_x, ellipse_y])
    rotation_matrix = U
    rotated_ellipse = rotation_matrix @ ellipse
    plt.plot(rotated_ellipse[0, :] + x2, rotated_ellipse[1, :] + y2, 'm')
    
    plt.pause(0.01)
    
    # Check if within tolerance
    if np.linalg.norm(error) < tolerance:
        print("Desired position reached!")
        break

# Plot manipulability over trajectory
plt.figure(2)
plt.plot(manipulability_values, 'r-')
plt.title('Manipulability across the trajectory')
plt.xlabel('Iteration')
plt.ylabel('Manipulability')
plt.grid(True)

plt.show()
