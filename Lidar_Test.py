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
MOVE_FORWARD = 'w0:1'
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

def correct_path(decoded_distances):
    """Adjusts the robot's path based on sensor readings to avoid close proximity to walls."""
    front = decoded_distances.get(0)
    left = decoded_distances.get(270)
    right = decoded_distances.get(90)
    frontleft = decoded_distances.get(315)
    frontright = decoded_distances.get(45)
    # Front - 使用 0°, 12°, 和 348° 的平均值
    # front = sum([
    #     decoded_distances.get(0),
    #     decoded_distances.get(12),
    #     decoded_distances.get(348)
    # ]) / 3

    # # Right - 使用 78°, 90°, 和 102° 的平均值
    # right = sum([
    #     decoded_distances.get(84),
    #     decoded_distances.get(96),
    #     decoded_distances.get(108)
    # ]) / 3

    # # Left - 使用 258°, 270°, 和 282° 的平均值
    # left = sum([
    #     decoded_distances.get(264),
    #     decoded_distances.get(276),
    #     decoded_distances.get(288)
    # ]) / 3
    if left < 15:
        print("Adjusting path: Too close to left wall, correcting right.")
        # Step 1: Small right turn
        packet_tx = packetize('r0: 15')
        if packet_tx:
            transmit(packet_tx)
            # [responses, time_rx] = receive()
            # print(f"Correct right turn command response: {response_string(CORRECT_RIGHT, responses)}")
            # time.sleep(0.3)  # Wait briefly to complete turn

        # packet_tx = packetize(CORRECT_RIGHT)
        # if packet_tx:
        #     transmit(packet_tx)
        #     # [responses, time_rx] = receive()
        #     # print(f"Correct right turn command response: {response_string(CORRECT_RIGHT, responses)}")
        #     # time.sleep(0.3)  # Wait briefly to complete turn

        # Step 2: Move forward by 1 inch
        packet_tx = packetize('w0:1')  # Move forward 1 inch
        if packet_tx:
            transmit(packet_tx)
            # [responses, time_rx] = receive()
            # print(f"Move forward 1 inch command response: {response_string('w0:1', responses)}")
            # time.sleep(0.4)  # Short wait to complete movement

        # Step 3: Small left turn to correct back
        packet_tx = packetize(CORRECT_LEFT)
        if packet_tx:
            transmit(packet_tx)
            # [responses, time_rx] = receive()
            # print(f"Correct back left command response: {response_string(CORRECT_LEFT, responses)}")
            # time.sleep(0.3)  # Wait briefly to complete turn back

    elif right < 15:
        print("Adjusting path: Too close to right wall, correcting left.")
        # Step 1: Small left turn
        packet_tx = packetize(CORRECT_LEFT)
        if packet_tx:
            transmit(packet_tx)
            # [responses, time_rx] = receive()
            # print(f"Correct left turn command response: {response_string(CORRECT_LEFT, responses)}")
            # time.sleep(0.3)  # Wait briefly to complete turn

        # Step 2: Move forward by 1 inch
        packet_tx = packetize('w0:1')  # Move forward 1 inch
        if packet_tx:
            transmit(packet_tx)
            # [responses, time_rx] = receive()
            # print(f"Move forward 1 inch command response: {response_string('w0:1', responses)}")
            # time.sleep(0.4)  # Short wait to complete movement

        # Step 3: Small right turn to correct back
        packet_tx = packetize(CORRECT_RIGHT)
        if packet_tx:
            transmit(packet_tx)
            # [responses, time_rx] = receive()
            # print(f"Correct back right command response: {response_string(CORRECT_RIGHT, responses)}")
            # time.sleep(0.3)  # Wait briefly to complete turn back

    elif left < 16 and right < 16:
        # Both sides have walls, try to center
        if abs(left - right) > 1:  # Tolerance for centering
            if right > left:
                print("Centering: Moving slightly left.")
                packet_tx = packetize(CORRECT_LEFT)
                if packet_tx:
                    transmit(packet_tx)
                    # [responses, time_rx] = receive()
                    # print(f"Centering command response: {response_string('CORRECT', responses)}")
                    # time.sleep(0.3)  # Wait briefly to complete turn
                # Step 2: Move forward by 0.5 inch
                packet_tx = packetize('w0:0.5')  # Move forward 0.5 inch
                if packet_tx:
                    transmit(packet_tx)
                    # [responses, time_rx] = receive()
                    # print(f"Move forward 0.5 inch command response: {response_string('w0:0.5', responses)}")
                    # time.sleep(0.4)  # Short wait to complete movement
                # Step 3: Small right turn to correct back
                packet_tx = packetize(CORRECT_RIGHT)
                if packet_tx:
                    transmit(packet_tx)
                    # [responses, time_rx] = receive()
                    # print(f"Correct back right command response: {response_string(CORRECT_RIGHT, responses)}")
                    # time.sleep(0.3)  # Wait briefly to complete turn back
            else:
                print("Centering: Moving slightly right.")
                packet_tx = packetize(CORRECT_RIGHT)
                
                if packet_tx:
                    transmit(packet_tx)
                    # [responses, time_rx] = receive()
                    # print(f"Centering command response: {response_string('CORRECT', responses)}")
                    # time.sleep(0.3)  # Wait briefly to complete turn
                # Step 2: Move forward by 0.5 inch
                packet_tx = packetize('w0:0.5')  # Move forward 0.5 inch
                if packet_tx:
                    transmit(packet_tx)
                    # [responses, time_rx] = receive()
                    # print(f"Move forward 0.5 inch command response: {response_string('w0:0.5', responses)}")
                    # time.sleep(0.4)  # Short wait to complete movement
                # Step 3: Small left turn to correct back
                packet_tx = packetize(CORRECT_LEFT)
                if packet_tx:
                    transmit(packet_tx)
                    # [responses, time_rx] = receive()
                    # print(f"Correct back left command response: {response_string(CORRECT_LEFT, responses)}")
                    # time.sleep(0.3)  # Wait briefly to complete turn back


def read_lidar_data():
    """
    Reads and parses LiDAR data from the Arduino.
    Expects data in a format like: '000a0b5c0d2e...'
    Each distance value is 4 characters (hex).
    """
    while True:
        if ser.in_waiting > 0:
            raw_data = ser.readline().decode('utf-8', errors='ignore').strip()

            # Check if the data length is 32 characters (8 distances, each 4 characters)
            if len(raw_data) == 32:
                try:
                    # Parse the data into distances
                    decoded_distances = {
                        target_angles[i // 4]: int(raw_data[i:i + 4], 16) / 10.0
                        for i in range(0, len(raw_data), 4)
                    }
                    return decoded_distances
                except ValueError:
                    continue  # Skip to the next loop iteration for invalid hex data



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

def make_decision():
    """Makes a decision based on sensor readings."""
    global TURN_COUNTER

    # Example of using the decoded distances in simulation:
    # decoded_distances = read_lidar_sensor()
    decoded_distances = read_lidar_data()

    front = decoded_distances.get(0)
    frontleft = decoded_distances.get(315)
    frontright = decoded_distances.get(45)
    left = decoded_distances.get(270)
    right = decoded_distances.get(90)
    # Front - 使用 0°, 12°, 和 348° 的平均值
    # front = sum([
    #     decoded_distances.get(0),
    #     decoded_distances.get(12),
    #     decoded_distances.get(348)
    # ]) / 3

    # # Right - 使用 78°, 90°, 和 102° 的平均值
    # right = sum([
    #     decoded_distances.get(84),
    #     decoded_distances.get(96),
    #     decoded_distances.get(108)
    # ]) / 3

    # # Left - 使用 258°, 270°, 和 282° 的平均值
    # left = sum([
    #     decoded_distances.get(264),
    #     decoded_distances.get(276),
    #     decoded_distances.get(288)
    # # ]) / 3
    # if   30 > front > 20:
    #     packet_tx = packetize("w0: 3")
    #     if packet_tx:
    #         transmit(packet_tx)

    # elif 20 > front > 15:
    #     packet_tx = packetize("w0: 2")


    # elif 10 < front < 15:
    #     packet_tx = packetize("w0: 1")
    #     if packet_tx:

    #     # if packet_tx:
    #         transmit(packet_tx)
        # packet_tx = packetize("w0: 1")

    
    # Case 1: Surrounded by walls on three sides (front, left, and right)
    if front <= 20 and right <= 20 and left <= 20:
        print("Surrounded on three sides. Turning 180 degrees to find a clear path.")
        packet_tx = packetize(TURN_AROUND)
        if packet_tx:
            transmit(packet_tx)
            # [responses, time_rx] = receive()
            # print(f"Turn around 180 degrees command response: {response_string(TURN_AROUND, responses)}")
            # time.sleep(0.5)
        TURN_COUNTER += 1  # Increment turn counter
    
    elif frontleft <= 16:
        print("315 obsticale")
        packet_tx = MOVE_BACKWARD
        if packet_tx:
            transmit(packet_tx)

        packet_tx = packetize('r0: 45')
        if packet_tx:
            transmit(packet_tx)
    
    elif frontright <= 16:
        print("45 obsticale")
        packet_tx = MOVE_BACKWARD
        if packet_tx:
            transmit(packet_tx)
        packet_tx = MOVE_BACKWARD
        if packet_tx:
            transmit(packet_tx)
            
        packet_tx = packetize('r0: -45')
        if packet_tx:
            transmit(packet_tx)
            # [responses, time_rx] = receive()
            # print(f"Turn around 180 degrees command response: {response_string(TURN_AROUND, responses)}")
            # time.sleep(0.5)

    # Case 2: Dead-end (turned around twice already)
    # if TURN_COUNTER >= MAX_TURN_AROUNDS:
    #     print("In a dead-end. Finding the clearest direction to move.")
    #     # Find the direction with the largest sensor reading (most open space)

    #     # time.sleep(1)
    #     max_reading = max(readings)
    #     max_index = readings.index(max_reading)
    #     print(max_reading)
    #     DEAD_END_TRIAL = 0
    #     while max_reading < 45:

    #         print(max_reading)
    #         print(max_index)
    #         if max_index == 0:
    #             direction = MOVE_FORWARD
    #         elif max_index == 1:
    #             direction = TURN_LEFT
    #         elif max_index == 2:
    #             direction = TURN_RIGHT
    #         elif max_index == 3:
    #             direction = TURN_AROUND

    #         packet_tx = packetize(direction)
    #         if packet_tx:
    #             transmit(packet_tx)
    #             [responses, time_rx] = receive()
    #             print(f"Dead-end correction command response: {response_string(direction, responses)}")
    #             DEAD_END_TRIAL += 1  # Reset counter after finding a way out
    #             time.sleep(1)  # Allow time to move/turn appropriately
    #         new_readings = read_sensors()
    #         max_reading = max(new_readings)
    #         max_index = new_readings.index(max_reading)
            
    #     TURN_COUNTER = 0
        # else:
        #     print("No clear path detected, stopping for safety.")
        #     packet_tx = packetize(STOP)
        #     if packet_tx:
        #         transmit(packet_tx)
        #         [responses, time_rx] = receive()
        #         print(f"Stop command response: {response_string(STOP, responses)}")
        #     return False  # Stop execution
    
    # Case 3: Obstacle detected directly in front
    if front <= OBSTACLE_THRESHOLD:
        print("Obstacle detected! Stopping immediately.")
        packet_tx = packetize(STOP)
        if packet_tx:
            transmit(packet_tx)
            # [responses, time_rx] = receive()
            # print(f"Stop command response: {response_string(STOP, responses)}")
            # time.sleep(0.2)
        
        packet_tx = packetize(MOVE_BACKWARD)
        if packet_tx:
            transmit(packet_tx)
            # [responses, time_rx] = receive()
            # print(f"Move backward command response: {response_string(MOVE_BACKWARD, responses)}")
            # time.sleep(1)

        # Determine the turn direction based on clearer space
        if right > left:
            turn_direction = TURN_LEFT
        else:
            turn_direction = TURN_RIGHT

        packet_tx = packetize(turn_direction)
        if packet_tx:
            transmit(packet_tx)
            # [responses, time_rx] = receive()
            # print(f"Turn command response: {response_string(turn_direction, responses)}")
            # time.sleep(1)
    else:
        # Case 4: Normal correction and movement
        correct_path(decoded_distances)
        packet_tx = packetize(MOVE_FORWARD)
        if packet_tx:
            transmit(packet_tx)
        # decoded_distances = read_lidar_data()

            # [responses, time_rx] = receive()
        # receive()
            # print(f"Move command response: {response_string(MOVE_FORWARD, responses)}")
    
    return True

RUN_RANDOM_MOVEMENT = True

# Configure the serial port (replace 'COM8' with your actual port)
ser = serial.Serial('COM8', 9600, timeout=1)
time.sleep(2)
print("Connected to Arduino")

try:
    while RUN_RANDOM_MOVEMENT:
        # if ser.in_waiting > 0:
        #     # Read a line from the serial buffer.
        #     raw_data = ser.readline().decode('utf-8', errors='ignore').strip()
        #     print(f"Raw data received: {raw_data}")

        decoded_distances = read_lidar_data()
        print(decoded_distances)
        distance_at_0 = decoded_distances.get(0)      # 返回 0 度方向的距离
        distance_at_45 = decoded_distances.get(45)    # 返回 45 度方向的距离
        distance_at_90 = decoded_distances.get(90)    # 返回 90 度方向的距离
        distance_at_135 = decoded_distances.get(135)  # 返回 135 度方向的距离
        distance_at_180 = decoded_distances.get(180)  # 返回 180 度方向的距离
        distance_at_225 = decoded_distances.get(225)  # 返回 225 度方向的距离
        distance_at_270 = decoded_distances.get(270)  # 返回 270 度方向的距离
        distance_at_315 = decoded_distances.get(315)  # 返回 315 度方向的距离

        # 打印每个角度的距离
        print(f"0度的距离: {distance_at_0}")
        print(f"45度的距离: {distance_at_45}")
        print(f"90度的距离: {distance_at_90}")
        print(f"135度的距离: {distance_at_135}")
        print(f"180度的距离: {distance_at_180}")
        print(f"225度的距离: {distance_at_225}")
        print(f"270度的距离: {distance_at_270}")
        print(f"315度的距离: {distance_at_315}")
        
        
        make_decision()
except KeyboardInterrupt:
    print("\nExiting...")
finally:
    ser.close()
    print("serial connection closed.")
    # if not make_decision():
        
    #     break
    # time.sleep(0.1)





    