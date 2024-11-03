import serial
import time

def send_lidar_request(ser):
    """Send the 'lr' command to request LIDAR data from Arduino."""
    command = "[lr]"  # The command to request LIDAR data
    ser.write(command.encode())
    print("Sent command to Arduino:", command)

def receive_lidar_data(ser, timeout=3):
    """Listen for LIDAR data from Arduino within a specified timeout and process it."""
    start_time = time.time()

    while True:
        if time.time() - start_time > timeout:
            print("Timeout: No valid LIDAR data received.")
            return None

        # Read a single line from Serial
        line = ser.readline().decode('utf-8').strip()

        # Check if the line starts with the expected LIDAR prefix
        if line.startswith("[lr:") and line.endswith("]"):
            # Remove the '[lr:' prefix and ']' suffix to get raw HEX data
            hex_data = line[4:-1]
            print("Received HEX data:", hex_data)

            # Process the HEX data into a list of distances in cm
            distances = []
            try:
                for i in range(0, len(hex_data), 4):  # Each distance value is 4 hex digits
                    hex_value = hex_data[i:i+4]
                    distance_mm = int(hex_value, 16)

                    # Convert mm to cm and filter invalid data
                    distance_cm = -1 if distance_mm == 65535 else distance_mm / 10
                    distances.append(distance_cm)
            except ValueError:
                print("Invalid HEX data received, skipping line...")
                continue  # Skip to the next line if there's an error in conversion

            # Print the list of distances in cm
            print("Distances in cm:", distances)

            # Check if all valid distances are within 50 cm
            if all(0 <= d < 50 for d in distances if d != -1):
                print("All distances are within 50 cm.")
                return distances  # Return distances if all are within 50 cm
            else:
                print("Not all distances are within 50 cm, continuing to pull data...")
        else:
            print("Ignoring non-LIDAR data:", line)

if __name__ == "__main__":
    # Configure the serial port (replace 'COM8' with your actual port)
    ser = serial.Serial('COM8', 115200, timeout=1)

    # Send the LIDAR request command
    send_lidar_request(ser)

    # Wait and receive the LIDAR data
    distances = receive_lidar_data(ser)
    if distances:
        print("Final 32 angle distances:", distances)
