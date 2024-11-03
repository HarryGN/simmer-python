import serial

def decode_tof_data(line):
    """Decode ToF sensor data and map to specific directions."""
    # Initialize a dictionary to hold the distances with directional labels
    direction_map = {
        'left': None,
        'left-front': None,
        'front': None,
        'right-front': None,
        'right': None
    }
    
    # Split the line into parts based on 'Sensor' identifiers
    sensors_data = line.split("Sensor")
    
    # Parse each sensor's data and map it to the direction
    for data in sensors_data:
        if data:  # Ignore any empty strings
            # Split by ':' to separate sensor number and distance
            sensor_info = data.split(":")
            if len(sensor_info) == 2:
                sensor_id = int(sensor_info[0].strip())
                distance = int(sensor_info[1].strip())

                # Convert distance to cm and mark invalid values
                if distance == 8191:
                    distance_cm = -1  # Mark -1 for out-of-range
                else:
                    distance_cm = distance / 10  # Convert to cm

                # Map sensor ID to the correct direction
                if sensor_id == 1:
                    direction_map['left'] = distance_cm
                elif sensor_id == 2:
                    direction_map['left-front'] = distance_cm
                elif sensor_id == 3:
                    direction_map['front'] = distance_cm
                elif sensor_id == 4:
                    direction_map['right-front'] = distance_cm
                elif sensor_id == 5:
                    direction_map['right'] = distance_cm

    # Print the results in a readable format
    for direction, distance in direction_map.items():
        if distance == -1:
            print(f"{direction.capitalize()}: Out of range")
        else:
            print(f"{direction.capitalize()}: {distance} cm")

# Example usage
def receive_tof_data():
    # Replace 'COM8' with your actual port
    with serial.Serial('COM8', 115200, timeout=1) as ser:
        while True:
            # Read a line of data
            line = ser.readline().decode('utf-8').strip()
            
            if line:
                print("Raw ToF data:", line)  # Print raw ToF data for reference
                decode_tof_data(line)  # Decode and print directional distances

# Call the function to start receiving and decoding ToF data
receive_tof_data()
