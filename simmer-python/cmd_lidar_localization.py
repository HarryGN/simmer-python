import serial
import time
def receive_feedback(line):
    """Process feedback from Arduino commands and convert to cmd_list format."""
    cmd_list = []
    commands = ["Forward", "Right", "Left", "Back"]

    # Split the line into individual commands
    parts = line.split()
    for i in range(0, len(parts), 2):
        cmd = parts[i]
        
        # 设置默认值
        if "Forward" in cmd:
            default_value = 2  # 默认前进 2 个单位
            direction = "w0"
        elif "Back" in cmd:
            default_value = 1  # 默认后退 2 个单位
            direction = "w0"
            default_value = -default_value  # 负值表示后退
        elif "Right" in cmd:
            default_value = 90  # 默认右转 90 度
            direction = "r0"
        elif "Left" in cmd:
            default_value = 90  # 默认左转 90 度
            direction = "r0"
            default_value = -default_value  # 负值表示左转
        else:
            continue  # 如果命令不在预期范围内，跳过

        # 检查是否有伴随的数值
        value = default_value
        if i + 1 < len(parts):
            try:
                value = float(parts[i + 1])  # 尝试将下一部分转换为数值
            except ValueError:
                pass  # 转换失败时，使用默认值

        # 添加到 cmd_list 中
        cmd_list.append(f"{direction}: {int(value)}")

    return cmd_list


def receive_lidar_data(line):
    """Parse a line of LIDAR data from Arduino and convert to list of distances."""
    if line.startswith("lr:"):
        hex_data = line[3:]
        distances = []
        try:
            for i in range(0, len(hex_data), 4):  # Each distance value is 4 hex digits
                hex_value = hex_data[i:i+4]
                distance_mm = int(hex_value, 16)

                # Convert mm to cm and filter invalid data
                distance_inch = -1 if distance_mm == 65535 else round(distance_mm / 25.4, 1)
                distances.append(distance_inch)
        except ValueError:
            print("Invalid HEX data received, skipping line...")
            return None  # Return None on invalid data

        print("Distances in inch:", distances)
        # Check if all valid distances are within 150 cm
        if all(0 <= d < 9999 for d in distances if d != -1):
            return distances  # Return distances if all are within 150 cm
        else:
            print("Not all distances are within 150 cm, continuing to pull data...")
            return None
    return None  # Return None if not a LIDAR data line


# Main execution loop
RUN_RANDOM_MOVEMENT = True

# Configure the serial port (replace 'COM8' with your actual port)
ser = serial.Serial('COM8', 115200, timeout=1)
time.sleep(2)
print("Connected to Arduino")

try:
    while RUN_RANDOM_MOVEMENT:
        # Read a single line from Serial
        line = ser.readline().decode('utf-8').strip()
        
        print("Raw data:", line)  # Print raw ToF data for reference
        # Process the line based on its content
        if line.startswith("lr:"):  # Check if it's LIDAR data
            distances = receive_lidar_data(line)
            if distances:
                print("Processed LIDAR data:", distances)
        else:  # Otherwise, process it as feedback
            cmd_list = receive_feedback(line)
            if cmd_list:
                print("Processed cmd_list:", cmd_list)

except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    ser.close()
