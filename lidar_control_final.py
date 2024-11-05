'''
    Predefined lists of cmd_sent
    Smaller griding
    lidar_data_matrix: (8, 105, 32)
        - 105 positions
        - 8 orientations per position
        - 32 data points per orientation
'''

import socket
import time
from datetime import datetime
import re
import numpy as np
import matplotlib.pyplot as plt
import serial

def decode_combined_value(combined_input):
    # 如果输入是嵌套列表，提取编码部分
    if isinstance(combined_input, list):
        # 假设结构为 [['li', '365083593092055087066110']]
        if len(combined_input) > 0 and isinstance(combined_input[0], list) and len(combined_input[0]) > 1:
            combined_str = combined_input[0][1]
        else:
            raise ValueError("输入结构无效，无法解码。")
    else:
        combined_str = str(combined_input)

    # 确保combined_str的长度是3的倍数（填充0）
    if len(combined_str) % 3 != 0:
        combined_str = combined_str.zfill((len(combined_str) // 3 + 1) * 3)
    
    # 分割组合字符串为每3个字符一组，并解码为浮点数
    decoded_distances = [
        int(combined_str[i:i+3]) / 10.0 for i in range(0, len(combined_str), 3)
    ]

    return decoded_distances

# Wrapper functions for communication
def transmit(data):
    if SIMULATE:
        transmit_tcp(data)
    else:
        transmit_serial(data)
    time.sleep(TRANSMIT_PAUSE)
def receive():
    if SIMULATE:
        return receive_tcp()
    else:
        return receive_serial()
def transmit_tcp(data):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((HOST, PORT_TX))
            s.send(data.encode('ascii'))
        except (ConnectionRefusedError, ConnectionResetError):
            print('Tx Connection was refused or reset.')
        except TimeoutError:
            print('Tx socket timed out.')
def receive_tcp():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s2:
        try:
            s2.connect((HOST, PORT_RX))
            response_raw = s2.recv(1024).decode('ascii')
            if response_raw:
                return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
            else:
                return [[False], datetime.now().strftime("%H:%M:%S")]
        except (ConnectionRefusedError, ConnectionResetError):
            print('Rx connection was refused or reset.')
        except TimeoutError:
            print('Response not received from robot.')

# Packetization and validation functions
def depacketize(data_raw: str):
    start = data_raw.find(FRAMESTART)
    end = data_raw.find(FRAMEEND)

    if (start >= 0 and end >= start):
        data = data_raw[start+1:end].replace(f'{FRAMEEND}{FRAMESTART}', ',').split(',')
        cmd_list = [item.split(':', 1) for item in data]

        for cmd_single in cmd_list:
            if len(cmd_single) == 1:
                cmd_single.append('')

        return cmd_list
    else:
        return [[False, '']]
def packetize(data: str):
    forbidden = [FRAMESTART, FRAMEEND, '\n']
    check_fail = any(char in data for char in forbidden)

    if not check_fail:
        return FRAMESTART + data + FRAMEEND

    return False
def response_string(cmds: str, responses_list: list):
    cmd_list = [item.split(':')[0] for item in cmds.split(',')]
    valid = validate_responses(cmd_list, responses_list)

    out_string = ''
    for item in zip(cmd_list, responses_list, valid):
        sgn = '=' if item[2] else '!='
        chk = '✓' if item[2] else 'X'
        cmd_response = item[1][0] if item[1] and len(item[1]) > 0 else "None"
        response_value = item[1][1] if item[1] and len(item[1]) > 1 else "None"
        out_string += f'cmd {item[0]} {sgn} {cmd_response} {chk}, response "{response_value}"\n'

    return out_string
def validate_responses(cmd_list: list, responses_list: list):
    valid = []
    for pair in zip(cmd_list, responses_list):
        valid.append(pair[1] and pair[0] == pair[1][0])
    return valid
def extract_numeric_value(response_string: str) -> float:
    pattern = r'response\s*"(\d+\.\d+)"'
    match = re.search(pattern, response_string)
    if match:
        return float(match.group(1))
    return None

############## Constant Definitions Begin ##############
HOST = '127.0.0.1'
PORT_TX = 61200
PORT_RX = 61201

FRAMESTART = '['
FRAMEEND = ']'
CMD_DELIMITER = ','

SIMULATE = True

if SIMULATE:
    SOURCE = 'SimMeR'
TRANSMIT_PAUSE = 0.04
OBSTACLE_THRESHOLD = 6

# Movement and turning commands
MOVE_FORWARD = 'w0:0.5'
MOVE_FRONT = 'w0:5'
MOVE_BACKWARD = 'w0:-0.5'
TURN_LEFT = 'r0:90'
TURN_RIGHT = 'r0:-90'
CORRECT_LEFT = 'r0:10'
CORRECT_RIGHT = 'r0:-10'
STOP = 'xx'

# track 180-degree turns and determine dead-ends
TURN_AROUND = 'r0:180'
TURN_COUNTER = 0
MAX_TURN_AROUNDS = 2  # Maximum allowed 180-degree turns before considering it a dead-end

previous_readings = [None, None, None, None]
stagnation_count = 0
MAX_STAGNATION = 5

def read_lidar_sensor():
    """
    Reads distance measurement from a single LiDAR sensor.
    Assumes that the sensor is identified by 'l0'.
    """
    sensor_id = 'li'
    packet_tx = packetize(sensor_id)
    if packet_tx:
        # print(f"Transmitting packet: {packet_tx}")
        transmit(packet_tx)

        # Receive the response packet
        result = receive()
        if result is not None:
            [responses, time_rx] = result
            # print(response_string(sensor_id, responses))
        # Decode the combined value
            decoded_distances = decode_combined_value(responses)
            # print(decoded_distances)
            # print(f"Received response at {time_rx}: {responses}")
        else:
            # If no reading is valid, return 'inf' to indicate no object detected
            print(f"LiDAR {sensor_id} reading: No object detected within range.")
    return decoded_distances

def is_stagnant(current_readings):
    global previous_readings, stagnation_count

    if previous_readings == current_readings:
        stagnation_count += 1
    else:
        stagnation_count = 0
    previous_readings = current_readings

    return stagnation_count >= MAX_STAGNATION
def correct_path(decoded_distances):
    """Adjusts the robot's path based on sensor readings to avoid close proximity to walls."""
    # Front - 使用 0°, 12°, 和 348° 的平均值
    front = decoded_distances.get(0)

    # Right - 使用 78°, 90°, 和 102° 的平均值
    right = decoded_distances.get(90)

    # Left - 使用 258°, 270°, 和 282° 的平均值
    left = decoded_distances.get(270)
    
    if left < 2.7:
        print("Adjusting path: Too close to left wall, correcting right.")
        # Step 1: Small right turn
        packet_tx = packetize(CORRECT_RIGHT)
        if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            print(f"Correct right turn command response: {response_string(CORRECT_RIGHT, responses)}")
            time.sleep(0.3)  # Wait briefly to complete turn

        # Step 2: Move forward by 1 inch
        packet_tx = packetize('w0:1')  # Move forward 1 inch
        if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            print(f"Move forward 1 inch command response: {response_string('w0:1', responses)}")
            time.sleep(0.4)  # Short wait to complete movement

        # Step 3: Small left turn to correct back
        packet_tx = packetize(CORRECT_LEFT)
        if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            print(f"Correct back left command response: {response_string(CORRECT_LEFT, responses)}")
            time.sleep(0.3)  # Wait briefly to complete turn back

    elif right < 2.7:
        print("Adjusting path: Too close to right wall, correcting left.")
        # Step 1: Small left turn
        packet_tx = packetize(CORRECT_LEFT)
        if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            print(f"Correct left turn command response: {response_string(CORRECT_LEFT, responses)}")
            time.sleep(0.3)  # Wait briefly to complete turn

        # Step 2: Move forward by 1 inch
        packet_tx = packetize('w0:1')  # Move forward 1 inch
        if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            print(f"Move forward 1 inch command response: {response_string('w0:1', responses)}")
            time.sleep(0.4)  # Short wait to complete movement

        # Step 3: Small right turn to correct back
        packet_tx = packetize(CORRECT_RIGHT)
        if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            print(f"Correct back right command response: {response_string(CORRECT_RIGHT, responses)}")
            time.sleep(0.3)  # Wait briefly to complete turn back

    elif left < 5.5 and right < 5.5:
        # Both sides have walls, try to center
        if abs(left - right) > 1:  # Tolerance for centering
            if right > left:
                print("Centering: Moving slightly left.")
                packet_tx = packetize(CORRECT_LEFT)
                if packet_tx:
                    transmit(packet_tx)
                    [responses, time_rx] = receive()
                    print(f"Centering command response: {response_string('CORRECT', responses)}")
                    time.sleep(0.3)  # Wait briefly to complete turn
                # Step 2: Move forward by 0.5 inch
                packet_tx = packetize('w0:0.5')  # Move forward 0.5 inch
                if packet_tx:
                    transmit(packet_tx)
                    [responses, time_rx] = receive()
                    print(f"Move forward 0.5 inch command response: {response_string('w0:0.5', responses)}")
                    time.sleep(0.4)  # Short wait to complete movement
                # Step 3: Small right turn to correct back
                packet_tx = packetize(CORRECT_RIGHT)
                if packet_tx:
                    transmit(packet_tx)
                    [responses, time_rx] = receive()
                    print(f"Correct back right command response: {response_string(CORRECT_RIGHT, responses)}")
                    time.sleep(0.3)  # Wait briefly to complete turn back
            else:
                print("Centering: Moving slightly right.")
                packet_tx = packetize(CORRECT_RIGHT)
                
                if packet_tx:
                    transmit(packet_tx)
                    [responses, time_rx] = receive()
                    print(f"Centering command response: {response_string('CORRECT', responses)}")
                    time.sleep(0.3)  # Wait briefly to complete turn
                # Step 2: Move forward by 0.5 inch
                packet_tx = packetize('w0:0.5')  # Move forward 0.5 inch
                if packet_tx:
                    transmit(packet_tx)
                    [responses, time_rx] = receive()
                    print(f"Move forward 0.5 inch command response: {response_string('w0:0.5', responses)}")
                    time.sleep(0.4)  # Short wait to complete movement
                # Step 3: Small left turn to correct back
                packet_tx = packetize(CORRECT_LEFT)
                if packet_tx:
                    transmit(packet_tx)
                    [responses, time_rx] = receive()
                    print(f"Correct back left command response: {response_string(CORRECT_LEFT, responses)}")
                    time.sleep(0.3)  # Wait briefly to complete turn back
def make_decision():
    """Makes a decision based on sensor readings."""
    global TURN_COUNTER

    # Example of using the decoded distances:
    decoded_distances = read_lidar_sensor()

    # Front - 使用 0°, 12°, 和 348° 的平均值
    front = decoded_distances.get(0)

    # Right - 使用 78°, 90°, 和 102° 的平均值
    right = decoded_distances.get(90)

    # Left - 使用 258°, 270°, 和 282° 的平均值
    left = decoded_distances.get(270)

    
    # Case 1: Surrounded by walls on three sides (front, left, and right)
    if front <= 7 and right <= 7 and left <= 7:
        print("Surrounded on three sides. Turning 180 degrees to find a clear path.")
        packet_tx = packetize(TURN_AROUND)
        if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            print(f"Turn around 180 degrees command response: {response_string(TURN_AROUND, responses)}")
            time.sleep(0.5)
        TURN_COUNTER += 1  # Increment turn counter
    
    # Case 3: Obstacle detected directly in front
    if front <= OBSTACLE_THRESHOLD:
        print("Obstacle detected! Stopping immediately.")
        packet_tx = packetize(STOP)
        if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            print(f"Stop command response: {response_string(STOP, responses)}")
            time.sleep(0.2)
        
        packet_tx = packetize(MOVE_BACKWARD)
        if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            print(f"Move backward command response: {response_string(MOVE_BACKWARD, responses)}")
            time.sleep(1)

        # Determine the turn direction based on clearer space
        if right > left:
            turn_direction = TURN_LEFT
        else:
            turn_direction = TURN_RIGHT

        packet_tx = packetize(turn_direction)
        if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            print(f"Turn command response: {response_string(turn_direction, responses)}")
            time.sleep(1)
    else:
        # Case 4: Normal correction and movement
        correct_path(decoded_distances)
        packet_tx = packetize(MOVE_FORWARD)
        if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            print(f"Move command response: {response_string(MOVE_FORWARD, responses)}")
    
    return True

RUN_RANDOM_MOVEMENT = False
while RUN_RANDOM_MOVEMENT:
    if not make_decision():
        break
    # time.sleep(0.1)


'''
A simple test function to read distance measurement from a LiDAR sensor.

# # 假设 decoded_distances 是解码后的字典
# # 示例调用读取所有角度的距离
# decoded_distances = read_lidar_sensor()
# print(decoded_distances)

# angles = [i * 11.25 for i in range(32)]

# for angle in angles:
#     distance = decoded_distances.get(angle)
#     print(f"{angle}度的距离: {distance}")
'''


"""
Function to read simulated lidar data for 105 positions in the 7x15 grid map.
    Assumptions:
        - 105 positions (rows in the file)
        - 8 orientations per position
        - 32 data points per orientation
    Return:
        lidar_data_matrix: (8, 105, 32)
"""
def process_lidar_data(file_path='lidar_data_sim_v5.txt'):
    parsed_data = []

    with open(file_path, 'r') as file:
        for line in file:
            # Strip and evaluate each line to parse as a list of lists
            line_data = eval(line.strip())
            parsed_data.append(line_data)

    # Initialize the 3D matrix with zeros to match the parsed data structure
    lidar_data_matrix = np.zeros((8, 288, 32))  # (orientations, positions, data points)

    # Fill the 3D matrix with parsed data
    for pos_idx, pos_data in enumerate(parsed_data):
        for orientation in range(8):
            lidar_data_matrix[orientation, pos_idx, :] = pos_data[orientation]

    # Display the shape of the matrix to confirm the structure
    # print("Shape of lidar_data_matrix:", lidar_data_matrix.shape)

    return lidar_data_matrix

def update_position(pos, orientation, distance):
    grid_distance = distance / 4  # Convert distance to grid units if necessary
    # 0 degree = point upward
    dx = grid_distance * np.sin(np.radians(orientation))
    dy = grid_distance * np.cos(np.radians(orientation))
    new_x = pos[0] - dx
    new_y = pos[1] - dy
    return (new_x, new_y)

def update_orientation(current_orientation, angle_change):
    new_orientation = (current_orientation + angle_change) % 360
    # Round to the nearest multiple of 45 degrees
    new_orientation = round(new_orientation / 45) * 45
    return new_orientation

def track_position_history(cmd_sent, start_orient):
    """Calculate the final position and orientation after executing all commands."""
    position = (0, 0)  # Initial (x, y) position
    orientation = start_orient  # Initial orientation in degrees
    
    for cmd in cmd_sent:
        if cmd.startswith('w0:'):
            distance = float(cmd.split(':')[1])
            position = update_position(position, orientation, distance)
        elif cmd.startswith('r0:'):
            angle_change = float(cmd.split(':')[1])
            orientation = update_orientation(orientation, angle_change)
    
    final_orientation = orientation
    # Round the position to avoid floating-point errors close to 0
    final_position = (round(position[0]), round(position[1]))
    print(f'Final pos = {final_position}, final orient = {final_orientation}')
    return final_position, final_orientation


def apply_translation(position_confidence, position):
    """Apply translation to the position_confidence matrix based on the final position."""
    rows, cols = position_confidence.shape
    # print('posi_confi BEFORE', position_confidence)
    # create_heatmap(position_confidence, 45) debug
    translated_confidence = np.full_like(position_confidence, 0.2)

    
    # Apply the translation
    x_shift, y_shift = position
    translated_confidence = np.roll(position_confidence, shift=-x_shift, axis=1)  # Shift left/right
    translated_confidence = np.roll(translated_confidence, shift=y_shift, axis=0) # Shift up/down
    # print('trans_matrix', translated_confidence)
    # create_heatmap(translated_confidence, 45) debug
    return translated_confidence

# Similarity between real and simulated lidar data
# Filter out -1 invalid readings
def calculate_similarity(real_data, simulated_data):
    """
    Calculate the similarity between real data and simulated data.
    Expands simulated_data to 9x17 if it is originally 7x15 by padding with 0.1 values.
    """
    # Check if simulated_data is 7x15 and expand it to 9x17 if necessary
    if simulated_data.shape == (7, 15):
        expanded_simulated_data = np.full((9, 17), 0.1)  # Create a 9x17 matrix with 0.1
        expanded_simulated_data[1:8, 1:16] = simulated_data  # Place the 7x15 data in the center
    else:
        expanded_simulated_data = simulated_data  # Use as-is if already 9x17

    # Create a mask to ignore -1 values in real_data
    valid_indices = real_data != -1
    filtered_real_data = np.array(real_data)[valid_indices]
    filtered_simulated_data = np.array(expanded_simulated_data)[valid_indices]
    
    # Calculate the inverse error as similarity
    error_sum = np.sum(np.abs(filtered_real_data - filtered_simulated_data))
    return 100 / (error_sum + 1e-5)  # Inverse error as similarity

# Update position confidence based on the highest similarity across all orientations
def localize_lidar(lidar_sim_matrix, real_data):
    rows, cols = 12, 24  # Updated maze dimensions in grid cells
    position_confidence = np.ones((rows, cols)) / (rows * cols)
    best_orientations = np.zeros((rows, cols), dtype=int)
    
    for r in range(rows):
        for c in range(cols):
            pos_index = r + (c * rows)  # Update pos_index for a 7x15 grid
            max_similarity = 0  # Track the highest similarity for each position
            best_orientation = 0  # Track the orientation with the highest similarity

            # Compare real data with all 8 orientation layers
            for orientation in range(8):
                # Access the full list of measurements for this position and orientation
                simulated_data = lidar_sim_matrix[orientation, pos_index, :]  # All 32 measurements for pos_index in orientation
                
                similarity = calculate_similarity(real_data, simulated_data)
                
                # Keep the maximum similarity found for this position
                if similarity > max_similarity:
                    max_similarity = similarity
                    best_orientation = orientation * 45  # Store the orientation angle

            # Multiply the position confidence by the maximum similarity score
            position_confidence[r, c] *= max_similarity
            best_orientations[r, c] = best_orientation
    
    # Normalize the confidence map
    min_confidence = position_confidence.min()
    max_confidence = position_confidence.max()
    position_confidence = (position_confidence - min_confidence) / (max_confidence - min_confidence)

    # Find the position with the maximum confidence
    max_confidence_index = np.unravel_index(np.argmax(position_confidence), position_confidence.shape)
    best_position_index = max_confidence_index[0] + (max_confidence_index[1] * rows)
    best_orientation = best_orientations[max_confidence_index]
    print(f"Best position index: {best_position_index}, Best orientation: {best_orientation}°")

    # Display updated heatmap (optional)
    # create_heatmap(position_confidence, best_orientation)
    # print("pos_conf", position_confidence.shape)

    return position_confidence, best_position_index, best_orientation



def accumulate_confidence_map(confidence_map, shifted_position_confidence, new_position_confidence):
    """Accumulate the new position confidence into the confidence map using multiplication."""
    confidence_map *= shifted_position_confidence 
    # print('previous confidence', shifted_position_confidence)

    confidence_map += new_position_confidence 
    # print('new confidence', new_position_confidence)

    # Normalize the confidence map to scale between 0 and 1
    min_conf = confidence_map.min()
    max_conf = confidence_map.max()
    # confidence_map = (confidence_map - min_conf) / (max_conf - min_conf) if max_conf > min_conf else confidence_map
    
    # All filter out block confidence
    confidence_map *= obstacle_filter
    return confidence_map


def improved_localize_lidar(lidar_sim_matrix, real_data, confidence_map, final_position, final_orientation):
    """Shift the confidence map and localize with the latest LiDAR scan on a finer 7x15 grid."""
    # Shift initial confidence based on the rover’s movements
    shifted_position_confidence = apply_translation(confidence_map, final_position)

    # Localize with the latest LiDAR data to create a new position confidence
    new_position_confidence, _, _ = localize_lidar(lidar_sim_matrix, real_data)

    # Accumulate the shifted and newly localized confidence maps
    confidence_map = accumulate_confidence_map(confidence_map, shifted_position_confidence, new_position_confidence)
    
    # Display the confidence map (optional)
    create_heatmap(confidence_map, final_orientation)

    return confidence_map


def create_heatmap(confidence_map, best_orientation):
   
    # # Automatic update figure
    # plt.clf()  # Clear the current figure
    # plt.imshow(confidence_map, cmap='viridis', interpolation='nearest')
    # plt.colorbar(label="Confidence")
    # plt.title("Position Confidence Heatmap")
    # plt.xlabel("Position X")
    # plt.ylabel("Position Y")
    # plt.xticks(range(confidence_map.shape[1]))
    # plt.yticks(range(confidence_map.shape[0]))
    
    # # Add dotted grid lines
    # plt.grid(color='gray', linestyle=':', linewidth=0.5)
    
    # # Find the most confident position (highest value in confidence_map)
    # max_confidence_index = np.unravel_index(np.argmax(confidence_map), confidence_map.shape)
    # row, col = max_confidence_index
    
    # # Calculate arrow direction based on orientation
    # arrow_dx = np.sin(np.radians(best_orientation))
    # arrow_dy = -np.cos(np.radians(best_orientation))  # Negative because Y-axis is inverted

    # # Draw an arrow at the center of the most confident block
    # plt.arrow(col, row, arrow_dx * 0.3, arrow_dy * 0.3, color='red', head_width=0.2, head_length=0.2)

    # plt.draw()  # Draw the updated plot
    # plt.pause(0.5)  # Pause for 0.5 seconds before moving to the next update
    
    # Automatic update figure
    plt.clf()  # Clear the current figure
    
    # Display the confidence map
    plt.imshow(confidence_map, cmap='viridis', interpolation='nearest')
    plt.colorbar(label="Confidence")
    plt.title("Position Confidence Heatmap")
    plt.xlabel("Position X (in feet)")
    plt.ylabel("Position Y (in feet)")

    # Calculate the tick positions to match the physical 4x8 feet maze size
    num_rows, num_cols = confidence_map.shape
    x_ticks = np.linspace(0, num_cols - 1, 9)  # 8-foot width
    y_ticks = np.linspace(0, num_rows - 1, 5)  # 4-foot height

    plt.xticks(ticks=x_ticks, labels=np.round(np.linspace(0, 8, 9), 1))  # 8 feet wide
    plt.yticks(ticks=y_ticks, labels=np.round(np.linspace(0, 4, 5), 1))  # 4 feet tall
    
    # Add dotted grid lines
    plt.grid(color='gray', linestyle=':', linewidth=0.5)
    
    # Find the most confident position (highest value in confidence_map)
    max_confidence_index = np.unravel_index(np.argmax(confidence_map), confidence_map.shape)
    row, col = max_confidence_index
    
    # Calculate arrow direction based on orientation
    arrow_dx = np.sin(np.radians(best_orientation))
    arrow_dy = -np.cos(np.radians(best_orientation))  # Negative because Y-axis is inverted

    # Draw an arrow at the center of the most confident block
    plt.arrow(col, row, arrow_dx * 0.3, arrow_dy * 0.3, color='red', head_width=0.2, head_length=0.2)

    plt.draw()  # Draw the updated plot
    plt.pause(0.5)  # Pause for 0.5 seconds before moving to the next update

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
        hex_data = line[3:-4]
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
            return None  # Return None on invalid data

        print("Distances in cm:", distances)
        # Check if all valid distances are within 150 cm
        if all(0 <= d < 150 for d in distances if d != -1):
            return distances  # Return distances if all are within 150 cm
        else:
            print("Not all distances are within 150 cm, continuing to pull data...")
            return None
    return None  # Return None if not a LIDAR data line



# ------------------------------------------ Localization Setup --------------------------------------------
plt.ion()
lidar_sim_matrix = process_lidar_data()     # Load the simulated data matrix (8, 105, 32)
confidence_map = np.ones((12, 24)) / (12 * 24) 
obstacle_filter = np.array([
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.2, 0.2, 0.2, 1, 1, 1, 0.2, 0.2, 0.2, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.2, 0.2, 0.2, 1, 1, 1, 0.2, 0.2, 0.2, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.2, 0.2, 0.2, 1, 1, 1, 0.2, 0.2, 0.2, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 0.2, 0.2, 0.2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 0.2, 0.2, 0.2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 0.2, 0.2, 0.2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
    [1, 1, 1, 0.2, 0.2, 0.2, 1, 1, 1, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1, 1, 1, 0.2, 0.2, 0.2, 1, 1, 1],
    [1, 1, 1, 0.2, 0.2, 0.2, 1, 1, 1, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1, 1, 1, 0.2, 0.2, 0.2, 1, 1, 1],
    [1, 1, 1, 0.2, 0.2, 0.2, 1, 1, 1, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1, 1, 1, 0.2, 0.2, 0.2, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.2, 0.2, 0.2, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.2, 0.2, 0.2, 1, 1, 1],
    [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.2, 0.2, 0.2, 1, 1, 1]
])

# Dynamic Localization
# # cmd_sent = ['r0:-45', 'w0:12', 'w0:12', "w0: 12", 'r0:90']
# cmd_sent1 = ['w0:12', 'w0:12', 'w0:12', 'r0:90', "w0: 12"]
# cmd_sent2 = ['r0:180', 'w0:12', 'r0:90', 'w0:12', 'w0:12']
# cmd_sent3 = ['r0:90', 'w0:12', 'w0:12', 'w0:12', 'r0:90']
# cmd_sent4 = ['w0:12',  'w0:12', 'w0:12', 'r0:90', 'w0:13.5', 'r0:-90', 'w0: 6.8']

# cmd_batch = [cmd_sent1, cmd_sent2, cmd_sent3, cmd_sent4]

LOOP_PAUSE_TIME = 1 # seconds

if __name__ == "__main__":
    # variables initialization
    cmd_sent = None
    initial_real_lidar = None
    new_cmd_received = False
    new_lidar_received = False
    try: 
        # 配置串口连接（将 'COM8' 替换为实际端口）
        ser = serial.Serial('COM8', 115200, timeout=1)
        time.sleep(2)
        print("Connected to Arduino")
        line = ser.readline().decode('utf-8').strip()
        print("Raw data:", line)  # 第一次原始数据
        if line.startswith("lr:"):  
            initial_real_lidar = receive_lidar_data(line)
            # 执行定位和热力图更新
            print("Both cmd_sent and initial_real_lidar are ready.")
            initial_position_confidence, start_pos, start_orient = localize_lidar(lidar_sim_matrix, initial_real_lidar)
            print(f"Start pos = {start_pos}, start orient = {start_orient}")
            
            confidence_map = accumulate_confidence_map(confidence_map, initial_position_confidence, initial_position_confidence) * 0.5
                
        while True:
            line = ser.readline().decode('utf-8').strip()
            print("Raw data:", line)  # 打印原始数据

            # 检查并处理 LIDAR 数据
            if line.startswith("lr:"):  
                distances = receive_lidar_data(line)
                if distances:
                    print("Processed LIDAR data:", distances)
                    new_real_data = distances  # 更新 LIDAR 数据
                    new_lidar_received = True  # 设置标志

            # 检查并处理命令反馈
            elif any(cmd in line for cmd in ["Forward", "Right", "Left", "Back"]):  
                cmd_list = receive_feedback(line)
                if cmd_list:
                    print("Processed cmd_list:", cmd_list)
                    cmd_sent = cmd_list  # 更新命令列表
                    new_cmd_received = True  # 设置标志

            # 当新 LIDAR 数据和命令列表均已接收时，进行处理并更新热力图
            if new_cmd_received and new_lidar_received:
                print("Moved 5 times, LiDAR starting scan")
                print("Localization starting here...")
                
                # 计算执行所有命令后的位置和方向
                final_position, final_orientation = track_position_history(cmd_sent, start_orient)
                
                # 进行 LiDAR 更新定位
                time.sleep(LOOP_PAUSE_TIME / 2)
                confidence_map = improved_localize_lidar(lidar_sim_matrix, new_real_data, confidence_map, final_position, final_orientation)
                
                # 更新热力图
                plt.ioff()
                # create_heatmap(confidence_map, final_orientation)  # 假设存在 create_heatmap 函数用于更新热力图
                plt.show()



                # 将起始方向更新为最终方向，准备下一循环
                start_orient = final_orientation

                # 重置标志以等待新数据
                new_cmd_received = False
                new_lidar_received = False

    except KeyboardInterrupt:
        print("Stopped by user.")
    finally:
        ser.close()



