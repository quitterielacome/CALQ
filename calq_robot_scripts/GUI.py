import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from tkinter import Tk, Entry, Button, Label, Frame
import numpy as np
import threading
import time
from kinematics import inverse_kinematics, forward_kinematics  # Import your kinematics functions
import csv
import os

# Initialize global variables
ser = None
positions = []  # To store (x, y) positions
angles = []  # To store angles (theta1, theta2)
current_positions = []  # For plotting
demanded_positions = []  # For plotting
motor_data = {}  # Dictionary to store parsed data
reached_target = False # Track whether the robot has reached the target
angle_tolerance = 5.0 # Define tolerance for reaching target
L1 = 115
L2 = 115
emergency_stop_activated = False # Global flag for emergency stop

# Function to log data
def log_data(x, y, theta1_deg, theta2_deg):
    file_exists = os.path.isfile("log_data.csv")
    with open("log_data.csv", mode='a', newline='') as file:
        writer = csv.writer(file)
        if not file_exists:
            writer.writerow(["X", "Y", "Theta1 (deg)", "Theta2 (deg)"])  # Write header if file is new
        writer.writerow([x, y, theta1_deg, theta2_deg])

# Function to set up the serial port
def setup_serial():
    global ser
    try:
        ser = serial.Serial('/dev/tty.usbmodem34B7DA6268982', 115200, timeout=1)  # Adjust the port as necessary
        time.sleep(2)  # Wait for the connection to establish
        ser.flush()  # Flush any existing data
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        ser = None  # Set ser to None if the connection fails

def send_command(x_str, y_str):
    global ser, current_positions, demanded_positions
    if ser is None:
        print("Serial port not open.")
        return
    if emergency_stop_activated:
        print("Emergency stop activated. Command not sent.")
        return  # Exit the function early

    try:
        x = float(x_str)
        y = float(y_str)

        if np.sqrt(x**2 + (y-7)**2) > (L1 + L2 + 1):
            print(f"Coordinates out of reach.")
            return

        theta1_deg, theta2_deg = inverse_kinematics(x, y)
        theta2_deg = -theta2_deg  # Adjust for motor direction

        if not (-360 <= theta1_deg <= 360) or not (-360 <= theta2_deg <= 360): #ask about angle limits of motor 2
            print(f"Computed angles out of range: {theta1_deg:.2f}, {theta2_deg:.2f}")
            return

        cmd_str = f"C{theta1_deg:.2f},{theta2_deg:.2f};\n"
        ser.write(cmd_str.encode())
        print(f"Sent: {cmd_str}")
        # Update demanded_positions with the single point
        demanded_positions = [(x, y)]  # Clear previous demands and set the new demand
        print(f"Demanded position updated: {demanded_positions}")
        # Clear previous positions to avoid redundant plotting
        
        # Log data
        log_data(x, y, theta1_deg, theta2_deg)
    except ValueError:
        print("Invalid input; enter valid coordinates or ensure point is in range.")
    except Exception as e:
        print(f"Error: {e}")


def read_data():
    global current_positions, reached_target
    while True:
        if ser and ser.in_waiting > 0:
            try:
                data_str = ser.readline().decode(errors='ignore').strip()
                print(f"Raw data received: {data_str}")

                # Parse data for motor demands and currents
                if "Motor1_Demand" in data_str and "Motor2_Demand" in data_str:
                    parts = data_str.split()
                    for part in parts:
                        if "Motor1_Demand" in part:
                            motor_data['demand1'] = float(part.split(":")[1])
                        elif "Motor1_Current" in part:
                            motor_data['current1'] = float(part.split(":")[1])
                        elif "Motor2_Demand" in part:
                            motor_data['demand2'] = float(part.split(":")[1])
                        elif "Motor2_Current" in part:
                            motor_data['current2'] = float(part.split(":")[1])

                if all(key in motor_data for key in ('demand1', 'current1', 'demand2', 'current2')):
                    # Check if the target is reached
                    if (abs(motor_data['demand1'] - motor_data['current1']) < angle_tolerance and 
                        abs(motor_data['demand2'] - motor_data['current2']) < angle_tolerance):
                        reached_target = True
                        print("Target position reached.")
                        
                    # Update current_positions with the current position
                    current_position = forward_kinematics(motor_data['current1'], motor_data['current2'])
                    current_positions.append(current_position)  # Append the current position
            except (IndexError, ValueError) as e:
                print(f"Error in received data format: {e}")


def draw_circle_threaded():
    """Retrieve user input and draw circle in a new thread."""
    try:
        diameter = float(diameter_input.get())
        points = int(points_input.get())
        center_x = float(center_x_input.get())
        center_y = float(center_y_input.get())
        
        # Start drawing in a separate thread
        threading.Thread(target=draw_circle, args=(diameter / 2, points, center_x, center_y), daemon=True).start()
    except ValueError:
        print("Invalid input values. Please enter numeric values.")

def draw_circle(radius=50, num_points=10, center_x=0, center_y=0):
    """Draw a circle using inverse kinematics and send commands to Arduino."""
    global reached_target, current_positions, demanded_positions, emergency_stop_activated
    points = [(center_x + radius * np.cos(angle), center_y + radius * np.sin(angle))
              for angle in np.linspace(0, 2 * np.pi, num_points, endpoint=False)]
    points.append(points[0])

    demanded_positions = points
    print(f'Demanded positions: {demanded_positions}')
    path_positions = []  # Accumulate positions for visualization

    for x, y in points:
        if emergency_stop_activated:
            print("Emergency stop activated. Stopping the circle drawing.")
            break
        try:
            # Calculate inverse kinematics to get the angles
            theta1, theta2 = inverse_kinematics(x, y)
            theta2 = -theta2
            # Send command using the calculated angles
            send_command(str(x), str(y))

            # Debugging output
            print(f"Moving to point ({x:.2f}, {y:.2f}) with angles θ1={theta1:.2f}, θ2={theta2:.2f}")
       
            # Reset reached_target before moving to the next point
            reached_target = False
    
            # Wait until the robot reaches the target position
            start_time = time.time()
            while not reached_target:
                time.sleep(0.1)
                if time.time() - start_time > 3:  # Timeout after 5 seconds
                    print(f"Timeout reaching point: ({x:.2f}, {y:.2f})")
                    break

            # Update path positions
            if reached_target:
                current_position = forward_kinematics(theta1, theta2)
                path_positions.append(current_position)
                print(f" This is current position list {current_position}")

            # Add delay between sending points
            time.sleep(1)  # Adjust the delay as needed

        except ValueError as e:
            print(f"Skipping unreachable point ({x:.2f}, {y:.2f}): {e}")
        except Exception as e:
            print(f"Error processing point ({x:.2f}, {y:.2f}): {e}")

    # Update current_positions to visualize the path
    # current_positions = path_positions
    print("Circle drawing complete!")


def update_plot(*args):
    """Update the real-time plot with the robot's current position."""
    if current_positions:
        x_coords, y_coords = zip(*current_positions)
        print(f"Plotting {len(current_positions)} points:")
        
        # For single-point plotting, just set data once
        if len(current_positions) == 1:
            print(f"Single point: ({x_coords[0]:.2f}, {y_coords[0]:.2f})")
            line1.set_data([x_coords[0]], [y_coords[0]])
        else:
            # Update the line for multi-point paths
            line1.set_data(x_coords, y_coords)

    if demanded_positions:
        if len(demanded_positions) == 1:
            # Single point: Plot as a cross
            demanded_x, demanded_y = demanded_positions[0]
            ax.scatter(demanded_x, demanded_y, color='blue', marker='x', label='Demanded Point', s=100)
        else:
            # Multi-point (circle): Plot as a line
            demanded_x, demanded_y = zip(*demanded_positions)
            line2.set_data(demanded_x, demanded_y)
            print('demanded position plotted')

    ax.relim()
    ax.autoscale_view()
    canvas.draw()
    return line2, line1


def emergency_stop():
    global demanded_theta1, demanded_theta2, emergency_stop_activated
    if ser:
        ser.write(b"S\n")  # Send stop command to Arduino
        demanded_theta1 = None
        demanded_theta2 = None
    emergency_stop_activated = True  # Activate the emergency stop flag
    print("Emergency Stop Activated!")

# Create main GUI window
root = Tk()
root.title("Arduino Communication")

# Frame for input fields and button
input_frame = Frame(root)
input_frame.pack(side="top", fill="x", pady=10)

# Input fields and send button
Label(input_frame, text="X Coordinate:").grid(row=0, column=0)
x_input = Entry(input_frame, width=10)
x_input.grid(row=0, column=1)

Label(input_frame, text="Y Coordinate:").grid(row=0, column=2)
y_input = Entry(input_frame, width=10)
y_input.grid(row=0, column=3)

send_button = Button(input_frame, text="Send", command=lambda: send_command(x_input.get(), y_input.get()))
send_button.grid(row=0, column=4)

Label(input_frame, text="Diameter:").grid(row=1, column=0)
diameter_input = Entry(input_frame, width=10)
diameter_input.grid(row=1, column=1)
diameter_input.insert(0, "60")  # Default value

Label(input_frame, text="Points:").grid(row=1, column=2)
points_input = Entry(input_frame, width=10)
points_input.grid(row=1, column=3)
points_input.insert(0, "30") 

Label(input_frame, text="Center X:").grid(row=2, column=0)
center_x_input = Entry(input_frame, width=10)
center_x_input.grid(row=2, column=1)
center_x_input.insert(0, "200")  # Default value

Label(input_frame, text="Center Y:").grid(row=2, column=2)
center_y_input = Entry(input_frame, width=10)
center_y_input.grid(row=2, column=3)
center_y_input.insert(0, "0") 

# Plot setup
fig, ax = plt.subplots()
ax.set_xlim(-250, 250)  
ax.set_ylim(-150, 350) 
ax.set_aspect('equal')  # Set equal scaling for x and y axes
line1, = ax.plot([], [], 'r-', label='Current')
line2, = ax.plot([], [], 'b-', label='Demanded')
ax.legend()

# Embed the plot in the Tkinter window
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side="top", fill="both", expand=True)

draw_circle_button = Button(input_frame, text="Draw Circle", command=draw_circle_threaded)
draw_circle_button.grid(row=1, column=4)

stop_button = Button(input_frame, text="Stop", command=emergency_stop, bg='red')
stop_button.grid(row=1, column=5)

# Set up serial before starting the data thread
setup_serial()

# Start data reading in a separate thread
data_thread = threading.Thread(target=read_data)
data_thread.daemon = True
data_thread.start()

# Set up real-time animation
ani = animation.FuncAnimation(fig, update_plot, blit=True, interval=50)

# Run GUI
root.protocol("WM_DELETE_WINDOW", root.quit)
root.mainloop()

# Close serial port on exit
if ser:
    ser.close()
