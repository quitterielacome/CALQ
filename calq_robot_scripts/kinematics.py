import numpy as np

# Arm lengths
L1 = 115
L2 = 115

# Offset for the first joint
OFFSET_Y = -70
ENCODER_OFFSET_THETA1 = -0

def forward_kinematics(theta1, theta2):
    """
    Calculate the end-effector position (x, y) for given joint angles,
    considering the first joint offset at (0, -5).

    Parameters:
        theta1 (float): The angle of the first joint in degrees.
        theta2 (float): The angle of the second joint in degrees.

    Returns:
        tuple: A tuple containing the x and y coordinates of the end-effector.
    """
    theta1 = theta1 - ENCODER_OFFSET_THETA1
    theta1_rad = np.radians(theta1)
    theta2_rad = np.radians(theta2)
    
    # Calculate position relative to the first joint
    x = L1 * np.cos(theta1_rad) + L2 * np.cos(theta1_rad + theta2_rad)
    y = L1 * np.sin(theta1_rad) + L2 * np.sin(theta1_rad + theta2_rad)
    
    # Adjust for the first joint offset
    y += OFFSET_Y
    
    return x, y

def inverse_kinematics(x, y):
    """
    Calculate the joint angles (theta1, theta2) for a given end-effector position,
    considering the first joint offset at (0, -5).

    Parameters:
        x (float): The x-coordinate of the target position.
        y (float): The y-coordinate of the target position.

    Returns:
        tuple: A tuple containing the angles theta1 and theta2 in degrees.

    Raises:
        ValueError: If the target point is out of reach for the robotic arm.
    """
    # Adjust for the first joint offset
    y -= OFFSET_Y

    # Calculate the distance to the target point
    d = np.sqrt(x**2 + (y)**2)
    
    # Check if the target is reachable
    if d > (L1 + L2):
        raise ValueError("Target point is out of reach.")
    
    # Compute angles in radians
    angle2 = np.arccos((d**2 - L1**2 - L2**2) / (2 * L1 * L2))
    angle1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(angle2), L1 + L2 * np.cos(angle2))
    
    # Convert radians to degrees
    angle1_deg = np.degrees(angle1) + ENCODER_OFFSET_THETA1
    angle2_deg = np.degrees(angle2)
    
    return angle1_deg, angle2_deg

def jacobian(theta1, theta2, L1, L2):
    """
    Compute the Jacobian matrix for a 2DOF planar robot arm.
    """
    dx_dtheta1 = -L1 * np.sin(theta1) - L2 * np.sin(theta1 + theta2)
    dx_dtheta2 = -L2 * np.sin(theta1 + theta2)

    dy_dtheta1 = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    dy_dtheta2 = L2 * np.cos(theta1 + theta2)

    J = np.array([[dx_dtheta1, dx_dtheta2],
                  [dy_dtheta1, dy_dtheta2]])

    return J

def differential_inverse_kinematics(x_d, y_d, theta1_init, theta2_init, max_iterations=100, tolerance=1e-3):
    """
    Use the Jacobian to iteratively solve for the joint angles to reach the target (x_d, y_d).
    """
    theta1, theta2 = theta1_init, theta2_init
    for _ in range(max_iterations):
        x, y = forward_kinematics(theta1, theta2)
        error = np.array([x_d - x, y_d - y])
        
        if np.linalg.norm(error) < tolerance:
            break

        J = jacobian(theta1, theta2)
        # Damped Least Squares (DLS) method
        damping_factor = 0.01
        J_damped = J @ np.linalg.inv(J.T @ J + damping_factor**2 * np.eye(2))
        delta_theta = J_damped @ error

        theta1 += delta_theta[0]
        theta2 += delta_theta[1]

    return theta1, theta2

# # Example usage
try:
    x_fk = 230 
    y_fk = -70
    theta1, theta2 = inverse_kinematics(x_fk, y_fk)
    print(f"Inverse Kinematics: Theta1 = {theta1:.2f}°, Theta2 = {theta2:.2f}°")
    x_fk, y_fk = forward_kinematics(theta1, theta2)
    print(f"Forward Kinematics: x = {x_fk:.2f}, y = {y_fk:.2f}") 

except ValueError as e:
    print(e)

