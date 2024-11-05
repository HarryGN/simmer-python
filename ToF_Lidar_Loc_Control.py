import socket
import time
from datetime import datetime
import re
import serial



# Wrapper functions
def transmit(data):
    '''Selects whether to use serial or tcp for transmitting.'''
    if SIMULATE:
        transmit_tcp(data)
    else:
        transmit_serial(data)
    time.sleep(TRANSMIT_PAUSE)

def receive():
    '''Selects whether to use serial or tcp for receiving.'''
    if SIMULATE:
        return receive_tcp()
    else:
        return receive_serial()

# TCP communication functions
def transmit_tcp(data):
    '''Send a command over the TCP connection.'''
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((HOST, PORT_TX))
            s.send(data.encode('ascii'))
        except (ConnectionRefusedError, ConnectionResetError):
            print('Tx Connection was refused or reset.')
        except TimeoutError:
            print('Tx socket timed out.')
        except EOFError:
            print('\nKeyboardInterrupt triggered. Closing...')

def receive_tcp():
    '''Receive a reply over the TCP connection.'''
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s2:
        try:
            s2.connect((HOST, PORT_RX))
            response_raw = s2.recv(1024).decode('ascii')
            if response_raw:
                # return the data received as well as the current time
                return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
            else:
                return [[False], datetime.now().strftime("%H:%M:%S")]
        except (ConnectionRefusedError, ConnectionResetError):
            print('Rx connection was refused or reset.')
        except TimeoutError:
            print('Response not received from robot.')

# serial communication functions
def transmit_serial(data):
    '''Transmit a command over a serial connection.'''
    # clear_serial()
    ser.write(data.encode('ascii'))

def receive_serial():
    '''Receive a reply over a serial connection.'''

    start_time = time.time()
    response_raw = ''
    while time.time() < start_time + TIMEOUT_serIAL:
        if ser.in_waiting:
            response_char = ser.read().decode('ascii')
            if response_char == FRAMEEND:
                response_raw += response_char
                break
            else:
                response_raw += response_char

    print(f'Raw response was: {response_raw}')

    # If response received, return it
    if response_raw:
        return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
    else:
        return [[False], datetime.now().strftime("%H:%M:%S")]

def clear_serial(delay_time: float = 0):
    '''Wait some time (delay_time) and then clear the serial buffer.'''
    if ser.in_waiting:
        time.sleep(delay_time)
        print(f'Clearing serial... Dumped: {ser.read(ser.in_waiting)}')

# Packetization and validation functions
def depacketize(data_raw: str):
    '''
    Take a raw string received and verify that it's a complete packet, returning just the data messages in a list.
    '''

    # Locate start and end framing characters
    start = data_raw.find(FRAMESTART)
    end = data_raw.find(FRAMEEND)

    # Check that the start and end framing characters are present, then return commands as a list
    if (start >= 0 and end >= start):
        data = data_raw[start+1:end].replace(f'{FRAMEEND}{FRAMESTART}', ',').split(',')
        cmd_list = [item.split(':', 1) for item in data]

        # Make sure this list is formatted in the expected manner
        for cmd_single in cmd_list:
            match len(cmd_single):
                case 0:
                    cmd_single.append('')
                    cmd_single.append('')
                case 1:
                    cmd_single.append('')
                case 2:
                    pass
                case _:
                    cmd_single = cmd_single[0:2]

        return cmd_list
    else:
        return [[False, '']]

def packetize(data: str):
    '''
    Take a message that is to be sent to the command script and packetize it with start and end framing.
    '''

    # Check to make sure that a packet doesn't include any forbidden characters (0x01, 0x02, 0x03, 0x04)
    forbidden = [FRAMESTART, FRAMEEND, '\n']
    check_fail = any(char in data for char in forbidden)

    if not check_fail:
        print(FRAMESTART + data + FRAMEEND)
        return FRAMESTART + data + FRAMEEND
        # print(FRAMESTART + data + FRAMEEND)

    return False

def response_string(cmds: str, responses_list: list):
    '''
    Build a string that shows the responses to the transmitted commands that can be displayed easily.
    '''
    # Validate that the command ids of the responses match those that were sent
    cmd_list = [item.split(':')[0] for item in cmds.split(',')]
    valid = validate_responses(cmd_list, responses_list)

    # Build the response string
    out_string = ''
    sgn = ''
    chk = ''
    for item in zip(cmd_list, responses_list, valid):
        if item[2]:
            sgn = '='
            chk = '✓'
        else:
            sgn = '!='
            chk = 'X'

        out_string = out_string + (f'cmd {item[0]} {sgn} {item[1][0]} {chk}, response "{item[1][1]}"\n')

    return out_string

def validate_responses(cmd_list: list, responses_list: list):
    '''
    Validate that the list of commands and received responses have the same command id's. Takes a
    list of commands and list of responses as inputs, and returns a list of true and false values
    indicating whether each id matches.
    '''
    valid = []
    for pair in zip(cmd_list, responses_list):
        if pair[1]:
            if pair[0] == pair[1][0]:
                valid.append(True)
            else:
                valid.append(False)
    return valid




############## Constant Definitions Begin ##############
### Network Setup ###
HOST = '127.0.0.1'      # The server's hostname or IP address
PORT_TX = 61200         # The port used by the *CLIENT* to receive
PORT_RX = 61201         # The port used by the *CLIENT* to send data

### serial Setup ###
BAUDRATE = 9600         # Baudrate in bps
PORT_serIAL = 'COM8'    # COM port identification
TIMEOUT_serIAL = 1      # serial port timeout, in seconds

### Packet Framing values ###
FRAMESTART = '['
FRAMEEND = ']'
CMD_DELIMITER = ','

### Set whether to use TCP (SimMeR) or serial (Arduino) ###
SIMULATE = False



############### Initialize ##############
### Source to display
if SIMULATE:
    SOURCE = 'SimMeR'
# else:
#     SOURCE = 'serial device ' + PORT_serIAL
# try:
#     SER = serial.Serial(PORT_SERIAL, BAUDRATE, timeout=TIMEOUT_SERIAL)
# except serial.SerialException:
#     print(f'Serial connection was refused.\nEnsure {PORT_SERIAL} is the correct port and nothing else is connected to it.')

### Pause time after sending messages
if SIMULATE:
    TRANSMIT_PAUSE = 0.1
else:
    TRANSMIT_PAUSE = 0.4


# Movement and turning commands
MOVE_FORWARD = 'w0:3'
MOVE_FRONT = 'w0:5'
MOVE_BACKWARD = 'w0:-2'
TURN_LEFT = 'r0:90'
TURN_RIGHT = 'r0:-90'
CORRECT_LEFT = 'r0:10'
CORRECT_RIGHT = 'r0:-10'
STOP = 'xx'

# track 180-degree turns and determine dead-ends
TURN_AROUND = 'r0:180'
TURN_COUNTER = 0
MAX_TURN_AROUNDS = 2  # Maximum allowed 180-degree turns before considering it a dead-end
OBSTACLE_THRESHOLD = 15

previous_readings = [None, None, None, None]
stagnation_count = 0
MAX_STAGNATION = 5
# Predefined target angles
target_angles = [0, 45, 90, 135, 180, 225, 270, 315]



def decode_combined_value(combined_input):
    '''
    解码组合字符串为对应角度的单独距离读数。
    每个距离由三个字符表示，并且保留一位小数。

    参数:
    - combined_input: str 或 list - 编码后的字符串或包含字符串的列表（例如：[['li', '365083593092055087066110']]）。

    返回:
    - dict，键为角度（0, 45, 90, ...），值为对应角度的解码距离（浮点数）。
    '''
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

    # 对应的角度列表（0度到315度，每45度一个方向）
    angles = [i for i in range(0, 360, 12)]
    

    # 创建字典，将角度和解码距离配对
    angle_distance_map = {angle: dist for angle, dist in zip(angles, decoded_distances)}

    return angle_distance_map





def extract_numeric_value(response_string: str) -> float:
    pattern = r'response\s*"(\d+\.\d+)"'
    match = re.search(pattern, response_string)
    if match:
        return float(match.group(1))
    return None




def read_sensors():
    
    readings = []
    for sensor_id in ['u0', 'u1', 'u2', 'u3']:
        packet_tx = packetize(sensor_id)
        if packet_tx:
            transmit(packet_tx)
            # [responses, time_rx] = receive()
            measured_distance = response_string(sensor_id, responses)
            reading = extract_numeric_value(measured_distance)
            if reading is not None:
                readings.append(reading)
            print(f"Ultrasonic {sensor_id} reading: {reading}")
    
    return readings if readings else [float('inf')] * 4

def is_stagnant(current_readings):
    global previous_readings, stagnation_count

    if previous_readings == current_readings:
        stagnation_count += 1
    else:
        stagnation_count = 0
    previous_readings = current_readings

    return stagnation_count >= MAX_STAGNATION


def receive_tof_data():
    # Replace 'COM8' with your actual port
    while True:
        # Read a line of data
        line = ser.readline().decode('utf-8').strip()

        # Ignore lines that do not contain "Sensor" or look like command responses
        if not line.startswith("Sensor"):
            print(f"Ignoring non-ToF data: {line}")
            continue  # Skip this line and continue reading

        if line:
            # print("Raw ToF data:", line)  # Print raw ToF data for reference
            
            # Decode ToF data
            direction_map = decode_tof_data(line)  # Decode and get directional distances

            # Verify if the direction_map has valid readings (non-None values)
            if any(value is not None for value in direction_map.values()):
                # print("Decoded ToF Distances by Direction:")
                # for direction, reading in direction_map.items():
                    # if reading == -1:
                    #     print(f"{direction.capitalize()}: Out of range")
                    # else:
                    #     print(f"{direction.capitalize()}: {reading} mm")
                return direction_map  # Return the map if valid data is found
            else:
                print("No valid ToF data found, continuing to read...")

def decode_tof_data(line):
    """Decode ToF sensor data and map to specific directions."""
    # Initialize a dictionary to hold the distances with directional labels
    direction_map = {
        'left': 999,
        'left-front': 999,
        'front': 999,
        'right-front': 999,
        'right': 999
    }

    # Split the line into parts based on 'Sensor' identifiers
    sensors_data = line.split("Sensor")

    # Parse each sensor's data and map it to the direction
    for data in sensors_data:
        if data:  # Ignore any empty strings
            # Split by ':' to separate sensor number and distance
            sensor_info = data.split(":")
            if len(sensor_info) == 2:
                try:
                    sensor_id = int(sensor_info[0].strip())
                    distance_text = sensor_info[1].strip()

                    # Check if the distance is numeric
                    if distance_text.isdigit():
                        distance = int(distance_text)
                        # Convert distance to cm and mark invalid values
                        distance_mm = -1 if distance == 8191 else distance  # Convert to cm
                    else:
                        # Handle non-numeric values (e.g., "Out of range")
                        distance_mm = 8191  # Use -1 to represent "Out of range"

                    # Map sensor ID to the correct direction
                    if sensor_id == 1:
                        direction_map['left'] = distance_mm
                    elif sensor_id == 2:
                        direction_map['left-front'] = distance_mm
                    elif sensor_id == 3:
                        direction_map['front'] = distance_mm
                    elif sensor_id == 4:
                        direction_map['right-front'] = distance_mm
                    elif sensor_id == 5:
                        direction_map['right'] = distance_mm
                except ValueError:
                    print("Invalid sensor data encountered, skipping entry...")
                    continue  # Skip invalid sensor data

    return direction_map  # Return the dictionary

def send_lidar_request(ser):
    """Send the 'lr' command to request LIDAR data from Arduino."""
    command = "[lr]"  # The command to request LIDAR data
    ser.write(command.encode())
    print("Sent command to Arduino:", command)

def receive_feedback(ser):
    """Process feedback from Arduino commands and convert to cmd_list format."""
    # 初始化空的 cmd_list
    cmd_list = []
    commands = ["Forward", "Right", "Left", "Back"]
    while True:

        line = ser.readline().decode('utf-8').strip()
        # 将命令反馈行分割成单个命令
        for cmd in line.split():
            # 处理 Forward 和 Back 命令，将它们转换为 w0 格式
            if "Forward" in cmd:
                # 提取数值并转换为 `w0: value` 格式
                value = int(cmd.split()[-1]) if cmd.split()[-1].isdigit() else 2  # 默认前进 2
                cmd_list.append(f"w0: {value}")
            elif "Back" in cmd:
                value = int(cmd.split()[-1]) if cmd.split()[-1].isdigit() else 2  # 默认后退 2
                cmd_list.append(f"w0: -{value}")
            # 处理 Right 和 Left 命令，将它们转换为 r0 格式
            elif "Right" in cmd:
                angle = int(cmd.split()[-1]) if cmd.split()[-1].isdigit() else 90  # 默认右转 90 度
                cmd_list.append(f"r0: {angle}")
            elif "Left" in cmd:
                angle = int(cmd.split()[-1]) if cmd.split()[-1].isdigit() else 90  # 默认左转 90 度
                cmd_list.append(f"r0: -{angle}")

        # print("Parsed cmd_list:", cmd_list)
    
    return cmd_list


def receive_lidar_data(ser):
    """Continuously listen for LIDAR data from Arduino until valid data is received."""
    while True:
        # send_lidar_request(ser)
        # Read a single line from Serial
        line = ser.readline().decode('utf-8').strip()

        # Check if the line starts with the expected LIDAR prefix
        if line.startswith("lr:"):
            # Remove the '[lr:' prefix and ']' suffix to get raw HEX data
            hex_data = line[3:0]
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
            if all(0 <= d < 150 for d in distances if d != -1):
                print("All distances are within 50 cm.")
                return distances  # Return distances if all are within 50 cm
            else:
                print("Not all distances are within 150 cm, continuing to pull data...")
        else:
            print("Ignoring non-LIDAR data:", line)


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
            print(response_string(sensor_id, responses))
        # Decode the combined value
            decoded_distances = decode_combined_value(responses)
            print(decoded_distances)
            print(f"Received response at {time_rx}: {responses}")
        else:
            # If no reading is valid, return 'inf' to indicate no object detected
            print(f"LiDAR {sensor_id} reading: No object detected within range.")
    return decoded_distances
def correct_path( left, diag_left, front, diag_right, right):
        
        # Map readings to respective directions
        # left = direction_map['left']
        # diag_left = direction_map['left-front']
        # front = direction_map['front']
        # diag_right = direction_map['right-front']
        # right = direction_map['right']
        # Correct path if too close to walls
        if (left < 55 or diag_left < 78) and right >= 55:
            print("Adjusting path: Too close to left wall, correcting right.")
            transmit(packetize("STOP"))
            transmit(packetize("r0: 5"))
            # transmit(packetize("w0: 1"))   # Move forward a little
            # transmit(packetize("r0: -5"))  # Correct back to stay parallel
            # cmd_list.extend(["w0: 1"])

        elif (right < 55 or diag_right < 78) and left >= 55:
            print("Adjusting path: Too close to right wall, correcting left.")
            transmit(packetize("STOP"))
            transmit(packetize("r0: -5"))

def make_decision(ser, cmd_list, left, diag_left, front, diag_right, right):
    """Makes a decision based on sensor readings and retrieves LIDAR data after 5 commands."""
    global TURN_COUNTER
    # Retrieve decoded distances from the ToF data
    # direction_map = receive_tof_data()
    # direction_map = receive_tof_data()
    # # # Map readings to respective directions
    # left = direction_map['left']
    # diag_left = direction_map['left-front']
    # front = direction_map['front']
    # diag_right = direction_map['right-front']
    # right = direction_map['right']
    # print("\nIndividual ToF Distance Readings:")
    # print(f"Left: {left if left != -1 else 'Out of range'} mm")
    # print(f"Left-front: {diag_left if diag_left != -1 else 'Out of range'} mm")
    # print(f"Front: {front if front != -1 else 'Out of range'} mm")
    # print(f"Right-front: {diag_right if diag_right != -1 else 'Out of range'} mm")
    # print(f"Right: {right if right != -1 else 'Out of range'} mm")



    # Case 1: Surrounded by walls on three sides (front, left, and right)
    if front <= 80 and left <= 80 and right <= 80:
        transmit(packetize("xx"))
        print("Surrounded on three sides. Turning 180 degrees to find a clear path.")
        transmit(packetize("w0: -2"))  # Move back to create space
        transmit(packetize("r0: 180"))  # Turn around
        cmd_list.extend(["w0: -2", "r0: 180"])
        time.sleep(1.5)
        TURN_COUNTER = TURN_COUNTER + 1
        # direction_map = receive_tof_data()


    # Case 2: Dead-end (turned around twice already)
    elif (left >= 35 or right >= 35) and TURN_COUNTER >= 2:
        print("Dead-end detected. Attempting to escape.")
        max_reading = max(left, right)
        transmit(packetize("w0: 4"))  # Move forward
        cmd_list.append("w0: 4")
        if max_reading == left:
            transmit(packetize("r0: -90"))  # Turn left if left side has more space
            cmd_list.append("r0: -90")
        else:
            transmit(packetize("r0: 90"))  # Turn right if right side has more space
            cmd_list.append("r0: 90")
        TURN_COUNTER = 0
        # direction_map = receive_tof_data()

    # Case 3: Obstacle detected directly in front
    elif front <= 80:
        print("Obstacle detected in front. Stopping and turning.")
        transmit(packetize("xx"))
        transmit(packetize("w0: -2"))  # Move back to create space
        cmd_list.append("w0: -2")
        if left > right:
            transmit(packetize("r0: -90"))  # Turn left if left side has more space
            cmd_list.append("r0: -90")
        else:
            transmit(packetize("r0: 90"))  # Turn right if right side has more space
            cmd_list.append("r0: 90")
        # direction_map = receive_tof_data()

    # Case 4: Diagonal obstacle on left
    elif diag_left <= 80:
        print("Diagonal obstacle on left. Correcting path.")
        transmit(packetize("xx"))
        transmit(packetize("w0: -2"))  # Move back slightly
        transmit(packetize("r0: 10"))  # Correct to the right
        # transmit(packetize("w0: 1"))   # Move forward a little
        # transmit(packetize("r0: -10"))  # Realign to remain parallel
        cmd_list.extend(["w0: -2", "w0: 1"])
        # direction_map = receive_tof_data()

    # Case 5: Diagonal obstacle on right
    elif diag_right <= 80:
        print("Diagonal obstacle on right. Correcting path.")
        transmit(packetize("xx"))
        transmit(packetize("w0: -2"))  # Move back slightly
        transmit(packetize("r0: -10"))  # Correct to the left
        # transmit(packetize("w0: 1"))   # Move forward a little
        # transmit(packetize("r0: 10"))  # Realign to remain parallel
        cmd_list.extend(["w0: -2", "w0: 1"])
        # direction_map = receive_tof_data()

    # Case 6: Clear path, move forward and correct path
    else:
        # direction_map = receive_tof_data()
        correct_path( left, diag_left, front, diag_right, right)
            # transmit(packetize("w0: 1"))   # Move forward a little
            # transmit(packetize("r0: 5"))   # Correct back to stay parallel
            # cmd_list.extend(["w0: 1"])

        # elif left < 4 and right < 4:
        #     if abs(left - right) > 0.5:
        #         if left > right:
        #             print("Centering: Moving slightly left.")
        #             transmit(packetize("r0: -5"))
        #             transmit(packetize("w0: 1"))   # Move forward a little
        #             transmit(packetize("r0: 5"))   # Realign to parallel
        #             cmd_list.extend(["w0: 1"])
        #         else:
        #             print("Centering: Moving slightly right.")
        #             transmit(packetize("r0: 5"))
        #             transmit(packetize("w0: 1"))   # Move forward a little
        #             transmit(packetize("r0: -5"))  # Realign to parallel
        #             cmd_list.extend(["w0: 1"])

        transmit(packetize("w0: 2"))  # Move forward to be replaced with cmmd 
        cmd_list.append("w0: 2")
        # direction_map = receive_tof_data()

# After executing 5 commands, request and receive LIDAR data
# send_lidar_request(ser)  # Send request for LIDAR data
# lidar_data = receive_lidar_data(ser)  # Receive the LIDAR data

# return cmd_list, lidar_data
    # time.sleep(1.5)
    return cmd_list


lookup_table = {
    # 被三面包围的情况
    ("Near", "Near", "Near", "Near", "Near"): ["w0: -2", "r0: 180"],

    # 前方有障碍物，但左右侧都有空隙
    ("Far", "Near", "Far", "Far", "Far"): ["xx", "w0: -2", "r0: 90"],
    
    # 前方有障碍物，且左侧有空隙
    ("Near", "Near", "Far", "Far", "Far"): ["r0: 90", "w0: 4"],
    ("Near", "Near", "Far", "Near", "Far"): ["r0: 90", "w0: 4"],

    # 前方有障碍物，且右侧有空隙
    ("Far", "Near", "Near", "Far", "Far"): ["r0: -90", "w0: 4"],
    ("Far", "Near", "Near", "Far", "Near"): ["r0: -90", "w0: 4"],

    # 左前对角有障碍物，选择轻微向右调整
    ("Near", "Far", "Far", "Near", "Far"): ["xx", "r0: 10"],

    # 右前对角有障碍物，选择轻微向左调整
    ("Far", "Far", "Near", "Far", "Near"): ["xx", "r0: -10"],

    # 全方位无障碍，选择前进
    ("Far", "Far", "Far", "Far", "Far"): ["w0: 2"],
}


# Helper function to determine "Near" or "Far"
def classify_distance(distance):
    return "Near" if distance <= 90 else "Far"

# Use the lookup table in your `make_decision` function
def make_decision_lut(ser, cmd_list, left, diag_left, front, diag_right, right):
    # Classify each sensor distance
    left_state = classify_distance(left)
    front_state = classify_distance(front)
    right_state = classify_distance(right)
    diag_left_state = classify_distance(diag_left)
    diag_right_state = classify_distance(diag_right)

    # Debug output to verify state classifications
    print("Sensor states - Left:", left_state, "Front:", front_state, "Right:", right_state,
          "Diag Left:", diag_left_state, "Diag Right:", diag_right_state)

    time.sleep(1)
    # Retrieve the action sequence based on sensor states
    action_sequence = lookup_table.get(
        (left_state, front_state, right_state, diag_left_state, diag_right_state),
        ["w0: 2"]  # Default action if no match found
    )

    # Debug output to verify action sequence
    print("Selected action sequence:", action_sequence)

    # Execute actions in sequence
    for action in action_sequence:
        transmit(packetize(action))
        cmd_list.append(action)

    return cmd_list



RUN_RANDOM_MOVEMENT = True

# Configure the serial port (replace 'COM8' with your actual port)
ser = serial.Serial('COM8', 115200, timeout=1)
time.sleep(2)
print("Connected to Arduino")

try:
    while RUN_RANDOM_MOVEMENT:
        line = ser.readline().decode('utf-8').strip()
        
        cmd_list = receive_feedback(line)
        print(cmd_list)
        receive_lidar_data(line)
        print(receive_lidar_data(line))
            

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    ser.close()
    print("serial connection closed.")
    # if not make_decision():
        
    #     break
    # time.sleep(0.1)





    