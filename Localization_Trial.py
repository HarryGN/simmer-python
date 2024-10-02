import socket
import time
from datetime import datetime
import re
import keyboard

# Maze configuration
MAZE_WIDTH = 32*3
MAZE_HEIGHT = 16*3


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
                # print(f"Raw data received: {response_raw}")  
                return [depacketize(response_raw), datetime.now().strftime("%H:%M:%S")]
            else:
                print("No data received from the sensor!")
                return [[None], datetime.now().strftime("%H:%M:%S")]
        except (ConnectionRefusedError, ConnectionResetError):
            print('Rx connection was refused or reset.')
        except TimeoutError:
            print('Response not received from robot.')



# Packetization and validation functions
def depacketize(data_raw: str):
   
    start = data_raw.find(FRAMESTART)
    end = data_raw.find(FRAMEEND)

    if start >= 0 and end >= start:
       
        data = data_raw[start + 1:end].replace(f'{FRAMEEND}{FRAMESTART}', ',').split(',')
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
TRANSMIT_PAUSE = 0.02
OBSTACLE_THRESHOLD = 2

# Movement and turning commands
MOVE_FORWARD = 'w0:0.5'
MOVE_3INCH = 'w0:3'
MOVE_5INCH = 'w0:5'
MOVE_10INCH = 'w0:10'
MOVE_2INCH = 'w0:2'
MOVE_BACKWARD = 'w0:-2'
TURN_LEFT = 'r0:-90'
TURN_RIGHT = 'r0:90'
CORRECT_LEFT = 'r0:-10'
CORRECT_RIGHT = 'r0:10'
STOP = 'xx'

# track 180-degree turns and determine dead-ends
TURN_AROUND = 'r0:180'
TURN_COUNTER = 0
MAX_TURN_AROUNDS = 2  # Maximum allowed 180-degree turns before considering it a dead-end

# Step size for robot movement in inches
move_step = 3


previous_readings = [None, None, None, None]
stagnation_count = 0
MAX_STAGNATION = 5

# Visulize constants
GRID_SIZE = 20
EMPTY_SPACE = ' '
OBSTACLE = '#'
ROBOT = 'R'
MAX_SENSOR_DISTANCE = 10 
# Initialize a 20x20 grid with empty spaces
def initialize_grid():
    return [[EMPTY_SPACE for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]

def print_grid(grid):
    """Print the current state of the grid."""
    for row in grid:
        print("".join(row))
    print("\n")


def read_sensors():
    # Eight readings, two for each direction: front_left, front_right, left_front, left_back, back_left, back_right, right_front, right_back
    readings = [None] * 4
    sensor_map = {
        'u0': 0,   # front left
        'u1': 1,  # front right
        'u2': 2,   # left front
        'u3': 3,  # left back
    }
    
    # Iterate over all sensor IDs
    for sensor_id in ['u0', 'u1', 'u2', 'u3']:
        packet_tx = packetize(sensor_id)
        if packet_tx:
            transmit(packet_tx)
            [responses, time_rx] = receive()
            for response in responses:
                sensor, value = response
                if sensor == sensor_id:
                    reading = extract_numeric_value(f'response "{value}"')
                    if reading is not None:
                        # Use the sensor_map to get the correct index for this sensor_id
                        sensor_index = sensor_map.get(sensor_id)
                        readings[sensor_index] = reading
                        print(f"Ultrasonic {sensor_id} reading: {readings[sensor_index]}")
    
    # Ensure we return valid float distances for each sensor
    return [reading if reading is not None else float('inf') for reading in readings]




# Constants for Keyboard Inputs
MOVE_3INCH_KEY = 'w3'
MOVE_5INCH_KEY = 'w5'
MOVE_10INCH_KEY = 'w10'
MOVE_2INCH_KEY = 'w2'
ADJUST_LEFT = "aa"
ADJUST_RIGHT = "dd"
BACK_KEY = 's'
LEFT_KEY = 'a'
RIGHT_KEY = 'd'
STOP_KEY = 'x'
def visualize_robot_and_walls(sensor_readings):
    front, right, left, back = sensor_readings
    sensor_grid = [
        "......    F:{:02}    ......".format(front),   # Front sensor
        ".. L:{:02}  O  R:{:02} ..".format(left, right),  # Left, center (O), and right sensors
        "......    B:{:02}   ......".format(back)    # Back sensor
    ]
    
    # Print the visualization
    for line in sensor_grid:
        print(line)

def handle_keyboard_input(command):
    if command == MOVE_3INCH_KEY:
        move_command = MOVE_3INCH
        packet_tx = packetize(move_command)
        if packet_tx:
            transmit(packet_tx)
            print("Moving forward 3 inches.")
        time.sleep(0.5)

    elif command == MOVE_5INCH_KEY:
        move_command = MOVE_5INCH
        packet_tx = packetize(move_command)
        if packet_tx:
            transmit(packet_tx)
            print("Moving forward 5 inches.")
        time.sleep(1)
    
    elif command == MOVE_10INCH_KEY:
        move_command = MOVE_10INCH
        packet_tx = packetize(move_command)
        if packet_tx:
            transmit(packet_tx)
            print("Moving forward 10 inches.")
        time.sleep(2)

    elif command == MOVE_2INCH_KEY:
        move_command = MOVE_2INCH
        packet_tx = packetize(move_command)
        if packet_tx:
            transmit(packet_tx)
            print("Moving forward 2 inches.")
        time.sleep(0.5)

    elif command == LEFT_KEY:
        packet_tx = packetize(TURN_LEFT)
        if packet_tx:
            transmit(packet_tx)
            print("Turning left.")
        time.sleep(1)

    elif command == RIGHT_KEY:
        packet_tx = packetize(TURN_RIGHT)
        if packet_tx:
            transmit(packet_tx)
            print("Turning right.")
        time.sleep(1)

    elif command == ADJUST_LEFT:
        print("Adjusting path: Too close to right wall, correcting left.")
        packet_tx = packetize(CORRECT_LEFT)
        if packet_tx:
            transmit(packet_tx)
            time.sleep(0.3)  # Wait briefly to complete turn left

        packet_tx = packetize(MOVE_FORWARD)
        if packet_tx:
            transmit(packet_tx)
            time.sleep(0.4)  # Short wait to complete movement

        packet_tx = packetize(CORRECT_RIGHT)
        if packet_tx:
            transmit(packet_tx)
            time.sleep(0.3)  # Wait briefly to complete turn back

    elif command == ADJUST_RIGHT:
        print("Adjusting path: Too close to left wall, correcting right.")
        packet_tx = packetize(CORRECT_RIGHT)
        if packet_tx:
            transmit(packet_tx)
            time.sleep(0.3)  # Wait briefly to complete turn right

        packet_tx = packetize(MOVE_FORWARD)
        if packet_tx:
            transmit(packet_tx)
            time.sleep(0.4)  # Short wait to complete movement

        packet_tx = packetize(CORRECT_LEFT)
        if packet_tx:
            transmit(packet_tx)
            time.sleep(0.3)  # Wait briefly to complete turn back

    else:
        print("Invalid command.")

    sensor_readings = read_sensors() 
    print(sensor_readings)
    visualize_robot_and_walls(sensor_readings)

def is_stagnant(current_readings):
    global previous_readings, stagnation_count

    if previous_readings == current_readings:
        stagnation_count += 1
    else:
        stagnation_count = 0
    previous_readings = current_readings

    return stagnation_count >= MAX_STAGNATION

def correct_path(readings):
    """Adjusts the robot's path based on sensor readings to avoid close proximity to walls."""
    front, left, right, back = readings

    if left < 2.7 and right >= 2.7:
        print("Adjusting path: Too close to left wall, correcting right.")
        packet_tx = packetize(CORRECT_RIGHT)
        if packet_tx:
            transmit(packet_tx)
            time.sleep(0.3)  # Wait briefly to complete turn

        packet_tx = packetize(MOVE_FORWARD)
        if packet_tx:
            transmit(packet_tx)
            time.sleep(0.4)  # Short wait to complete movement

        packet_tx = packetize(CORRECT_LEFT)
        if packet_tx:
            transmit(packet_tx)
            time.sleep(0.3)  # Wait briefly to complete turn back

    elif right < 2.7 and left >= 2.7:
        print("Adjusting path: Too close to right wall, correcting left.")
        packet_tx = packetize(CORRECT_LEFT)
        if packet_tx:
            transmit(packet_tx)
            time.sleep(0.3)  # Wait briefly to complete turn

        packet_tx = packetize(MOVE_FORWARD)
        if packet_tx:
            transmit(packet_tx)
            time.sleep(0.4)  # Short wait to complete movement

        packet_tx = packetize(CORRECT_RIGHT)
        if packet_tx:
            transmit(packet_tx)
            time.sleep(0.3)  # Wait briefly to complete turn back

def make_decision():
    """Makes a decision based on sensor readings."""
    global TURN_COUNTER
    
    readings = read_sensors()

    if is_stagnant(readings):
        print("Sensor readings are stagnant. Stopping execution.")
        return False  # Stop execution if stagnant

    front_reading = readings[0]
    left_reading = readings[1]
    right_reading = readings[2]
    back_reading = readings[3]
    
    # Case 1: Surrounded by walls on three sides (front, left, and right)
    if front_reading <= 4 and left_reading <= 4 and right_reading <= 4:
        print("Surrounded on three sides. Turning 180 degrees to find a clear path.")
        packet_tx = packetize(TURN_AROUND)
        if packet_tx:
            transmit(packet_tx)
            time.sleep(0.5)
        TURN_COUNTER += 1  # Increment turn counter
    
    # Case 2: Obstacle detected directly in front
    elif front_reading <= OBSTACLE_THRESHOLD:
        print("Obstacle detected! Stopping immediately.")
        packet_tx = packetize(STOP)
        if packet_tx:
            transmit(packet_tx)
            time.sleep(0.2)
        
        packet_tx = packetize(MOVE_BACKWARD)
        if packet_tx:
            transmit(packet_tx)
            time.sleep(1)

        if left_reading > right_reading:
            turn_direction = TURN_LEFT
        else:
            turn_direction = TURN_RIGHT

        packet_tx = packetize(turn_direction)
        if packet_tx:
            transmit(packet_tx)
            time.sleep(1)
    
    # Case 3: Normal correction and movement
    else:
        correct_path(readings)
        packet_tx = packetize(MOVE_FORWARD)
        if packet_tx:
            transmit(packet_tx)
            time.sleep(0.5)
    
    return True

# Main loop with keyboard control
RUN_RANDOM_MOVEMENT = False
RUN_KEYBOARD_CONTROL = True  # New flag to control keyboard input mode

while RUN_KEYBOARD_CONTROL:
    command = input("Command: ").strip().lower()
    if command == 'q':
        print("Exiting...")
        running = False
    else:
        handle_keyboard_input(command)

while RUN_RANDOM_MOVEMENT:
    if not make_decision():
        break
    time.sleep(0.1)

