import serial
import time
import matplotlib.pyplot as plt
import numpy as np

# Configure the serial port (replace 'COM8' with your actual port)
ser = serial.Serial('COM8', 9600, timeout=1)

# Allow time for the connection to establish
time.sleep(2)
print("Connected to Arduino")

# Predefined target angles
target_angles = [0, 45, 90, 135, 180, 225, 270, 315]

def parse_lidar_data(data):
    """
    Parse the LiDAR distance data from Arduino.
    Expects data in a format like: '000a0b5c0d2e...'
    Each distance value is 4 characters (hex).
    """
    decoded_distances = {}
    try:
        # Ensure the data length is 32 characters (8 distances, each 4 characters)
        if len(data) != 32:
            print(f"Unexpected data length: {len(data)}. Expected 32 characters.")
            return decoded_distances

        # Iterate through the data in chunks of 4 characters (hex distances)
        for i in range(0, len(data), 4):
            distance_hex = data[i:i + 4]

            # Convert distance from hex to integer (in mm)
            distance_mm = int(distance_hex, 16)
            distance_cm = distance_mm / 10.0  # Convert to cm

            # Map the distance to the corresponding target angle
            angle = target_angles[i // 4]
            decoded_distances[angle] = distance_cm

    except ValueError as e:
        print(f"Error parsing data: {data}, Error: {e}")
    
    return decoded_distances

def update_plot(lidar_data):
    """
    Update the LiDAR data on a polar plot.
    """
    angles = np.deg2rad(list(lidar_data.keys()))  # Convert angles to radians for the polar plot
    distances = list(lidar_data.values())

    ax.clear()
    ax.set_theta_zero_location('N')  # Set 0 degrees at the top (North)
    ax.set_theta_direction(-1)  # Clockwise direction
    ax.set_ylim(0, 10)  # Set the range of distances to be between 0 and 10 cm

    # Plot the points
    ax.scatter(angles, distances, c='red', s=50, label='LiDAR distances')
    ax.plot(angles, distances, c='blue', alpha=0.5)  # Connect the points with lines for better visualization

    # Add labels and title
    ax.set_title('LiDAR Distance Readings')
    ax.set_rticks([2, 4, 6, 8, 10])  # Distance ticks for clarity
    plt.legend(loc='upper right')

    # Draw the updated plot
    plt.draw()
    plt.pause(0.001)  # Pause for a very short time to allow the plot to update

# Set up the plot for real-time updates
plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(6, 6))

try:
    while True:
        if ser.in_waiting > 0:
            # Read a line from the serial buffer.
            raw_data = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"Raw data received: {raw_data}")

            # Parse the received data.
            lidar_data = parse_lidar_data(raw_data)
            print(f"Parsed LiDAR data: {lidar_data}")

            # Update the plot if we have valid data.
            if len(lidar_data) == len(target_angles):
                update_plot(lidar_data)

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    ser.close()
    plt.ioff()  # Turn off interactive mode
    plt.show()  # Keep the last frame displayed after the loop ends
    print("Serial connection closed.")
