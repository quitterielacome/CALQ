import matplotlib.pyplot as plt
import numpy as np
import time

# Parameters
time_steps = 100  # Number of time steps to simulate
normal_current = 0.3  # Normal current in amps
collision_current = 1.2  # Current during collision in amps
collision_time = 60  # Time step when collision occurs
current_threshold = 0.5  # Current threshold for detecting collisions

# Arrays to store data
time_data = np.arange(0, time_steps, 1)
current_data = []
collision_detected = []

# Simulate current readings
for t in time_data:
    if t < collision_time:
        current = np.random.normal(normal_current, 0.05)  # Normal current with noise
    else:
        current = np.random.normal(collision_current, 0.1)  # Collision current with noise

    current_data.append(current)

    # Check for collision
    if current > current_threshold:
        collision_detected.append(1)
    else:
        collision_detected.append(0)

# Visualization
plt.figure(figsize=(10, 6))
plt.plot(time_data, current_data, label="Motor Current (A)")
plt.axhline(y=current_threshold, color="r", linestyle="--", label="Collision Threshold")
plt.fill_between(
    time_data, 0, max(current_data), where=np.array(collision_detected) == 1,
    color="red", alpha=0.3, label="Collision Detected"
)
plt.title("Robot Current Simulation with Collision Detection")
plt.xlabel("Time Steps")
plt.ylabel("Current (A)")
plt.legend()
plt.grid()
plt.show()