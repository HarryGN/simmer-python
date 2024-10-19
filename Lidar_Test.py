import serial
import matplotlib.pyplot as plt
import numpy as np
import time

# Configure the serial port (change 'COM3' to your serial port)
ser = serial.Serial('COM3', 115200, timeout=1)

# Initialize data storage for 36 points (every 10 degrees)
angles = [i * 10 for i in range(36)]  # 0, 10, 20, ..., 350
distances = [5] * 36  # Initialize with the minimum distance (5 cm)

# Create a polar plot
plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
sc = ax.scatter([], [], s=5)

# Function to update the plot
def update_plot(angles, distances):
    ax.clear()
    ax.set_theta_zero_location('N')  # Set zero degrees to be at the top (North)
    ax.set_theta_direction(-1)  # Set clockwise direction
    ax.set_rlim(5, 30)  # Set distance limit (5 to 30 cm)

    # Convert angles to radians for the polar plot
    angles_rad = np.deg2rad(angles)
    sc = ax.scatter(angles_rad, distances, s=50, c='red')

    # Draw the plot
    plt.draw()
    plt.pause(0.001)

try:
    buffer = ""
    last_update = time.time()
    update_interval = 0.1  # Reduce update interval for faster refresh

    while True:
        # Read a larger chunk of data from the serial buffer
        if ser.in_waiting > 0:
            buffer += ser.read(ser.in_waiting).decode('utf-8', errors='ignore')

            # Split the buffer into lines
            lines = buffer.split('\n')
            buffer = lines[-1]  # Save the last incomplete line back to the buffer

            # Process each complete line
            new_data = False  # Flag to track if new data was processed
            for line in lines[:-1]:
                line = line.strip()

                # Check for new scan data
                if "New Scan" in line:
                    print("Received new scan data.")

                # Parse angle and distance data
                if "Angle" in line:
                    parts = line.split()
                    try:
                        angle = float(parts[1].strip("°"))  # Extract angle as float for non-integer values
                        distance = float(parts[3].strip("mm")) / 10.0  # Convert distance to cm

                        # Clamp the distance between 5 and 30 cm
                        distance = max(5, min(distance, 30))

                        # Find the closest angle in the target angles list
                        closest_index = min(range(len(angles)), key=lambda i: abs(angles[i] - angle))
                        distances[closest_index] = distance
                        new_data = True  # Mark that new data was processed

                    except (ValueError, IndexError) as e:
                        print(f"Error parsing line: {line}. Error: {e}")

            # Update the plot only if new data was processed and enough time has passed
            if new_data and time.time() - last_update >= update_interval:
                update_plot(angles, distances)
                last_update = time.time()

except KeyboardInterrupt:
    print("Program interrupted.")
finally:
    ser.close()  # Close the serial connection
