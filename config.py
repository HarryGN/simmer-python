'''
This file stores the configuration information for the simulator.
'''
# Imports
import pygame.math as pm
from devices.motors import MotorSimple
from devices.ultrasonic import Ultrasonic
from devices.lidar import Lidar
from devices.gyroscope import Gyroscope
from devices.compass import Compass
from devices.infrared import Infrared
from devices.drive import Drive

# Control Flags and Setup
rand_error = False          # Use either true random error generator (True) or repeatable error generation (False)
rand_bias = True           # Use a randomized, normally distributed set of bias values for drives (placeholder, not implemented)
bias_strength = [0.05, 1]   # How intense the random drive bias is, if enabled (placeholder, not implemented)

# Network configuration for sockets
host = '127.0.0.1'
port_rx = 61200
port_tx = 61201
timeout = 300
str_encoding = 'ascii'
frame_start = '['
frame_end = ']'

# General communication settings
round_digits = 3

# Block information
block_position = [66, 5]        # Block starting location
block_rotation = 0              # Block rotation (deg)
block_size = 3                  # Block side length in inches

# Robot information
robot_start_position = [6, 42]  # Robot starting location (in)
robot_start_rotation = 180      # Robot starting rotation (deg)
robot_width = 6                 # Robot width in inches
robot_height = 6                # Robot height in inches
robot_outline = [               # Robot outline, relative to center position
                pm.Vector2(-2.875,-4),
                pm.Vector2(-2.875,2.75),
                pm.Vector2(-1.655,4),
                pm.Vector2(1.655,4),
                pm.Vector2(2.875,2.75),
                pm.Vector2(2.875,-4)
                ]

# Maze definition information
wall_segment_length = 12    # Length of maze wall segments (inches)
floor_segment_length = 3    # Size of floor pattern squares (inches)
walls = [[3,3,1,1,0,2,0,2],
         [3,3,0,1,1,1,1,1],
         [1,0,2,0,0,1,0,1],
         [1,1,1,1,1,1,0,2]] # Matrix to define the maze walls
floor_seed = 5489           # Randomization seed for generating correct floor pattern
maze_dim_x = len(walls[0])*wall_segment_length
maze_dim_y = len(walls)*wall_segment_length

# Graphics information
frame_rate = 60             # Target frame rate (Hz)
ppi = 12                    # Number of on-screen pixels per inch on display
border_pixels = floor_segment_length * ppi  # Size of the border surrounding the maze area

background_color = (43, 122, 120)
wall_thickness = 0.25       # Thickness to draw wall segments, in inches
wall_color = (255, 0, 0)    # Tuple with wall color in (R,G,B) format
robot_thickness = 0.25      # Thickness to draw robot perimeter, in inches
robot_color = (0, 0, 255)   # Tuple with robot perimeter color in (R,G,B) format
block_thickness = 0.25      # Thickness to draw robot perimeter, in inches
block_color = (127, 127, 0) # Tuple with robot perimeter color in (R,G,B) format

### DEVICE CONFIGURATION ###
# Motors
m0_info = {
 'id': 'm0',
 'position': [3.125, 0],
 'rotation': 0,
 'visible': True
}
m1_info = {
 'id': 'm1',
 'position': [-3.125, 0],
 'rotation': 0,
 'visible': True
}
motors = {
 'm0': MotorSimple(m0_info),
 'm1': MotorSimple(m1_info)
}

# Drives
w0_info = {
    'id': 'w0',
    'position': [0, 0],
    'rotation': 0,
    'visible': False,
    'velocity': [0, 10],
    'ang_velocity': 0,
    'motors': [motors['m0'], motors['m1']],
    'motor_direction': [1, 1],
    'bias': {'x': 0, 'y': 0, 'rotation': 0},
    'error': {'x': 0, 'y': 0, 'rotation': 0}
}

d0_info = {
    'id': 'd0',
    'position': [0, 0],
    'rotation': 0,
    'visible': False,
    'velocity': [-10, 0],
    'ang_velocity': 0,
    'motors': [motors['m0'], motors['m1']],
    'motor_direction': [1, 1],
    'bias': {'x': 0, 'y': 0, 'rotation': 0},
    'error': {'x': 0, 'y': 0, 'rotation': 0}
}

r0_info = {
    'id': 'r0',
    'position': [0, 0],
    'rotation': 0,
    'visible': False,
    'velocity': [0, 0],
    'ang_velocity': 120,
    'motors': [motors['m0'], motors['m1']],
    'motor_direction': [1, -1],
    'bias': {'x': 0, 'y': 0, 'rotation': 0},
    'error': {'x': 0, 'y': 0, 'rotation': 0}
}

drives = {
    'w0': Drive(w0_info),
    'd0': Drive(d0_info),
    'r0': Drive(r0_info)
}

# Ultrasonic sensors positioned at front, left, right, and back of the robot
u0_info = {
 'id': 'u0',
 'position': [0, 3.5],  # Front
 'height': 1,
 'rotation': 0,
 'outline': [
 pm.Vector2(-1, -0.5),
 pm.Vector2(-1, 0.5),
 pm.Vector2(1, 0.5),
 pm.Vector2(1, -0.5)
 ],
 'visible': True,
 'visible_measurement': True
}
u1_info = {
 'id': 'u1',
 'position': [-2.5, 0],  # Left
 'height': 1,
 'rotation': 90,
 'outline': [
 pm.Vector2(-1, -0.5),
 pm.Vector2(-1, 0.5),
 pm.Vector2(1, 0.5),
 pm.Vector2(1, -0.5)
 ],
 'visible': True,
 'visible_measurement': True
}
u2_info = {
 'id': 'u2',
 'position': [2.5, 0],  # Right
 'height': 1,
 'rotation': -90,
 'outline': [
 pm.Vector2(-1, -0.5),
 pm.Vector2(-1, 0.5),
 pm.Vector2(1, 0.5),
 pm.Vector2(1, -0.5)
 ],
 'visible': True,
 'visible_measurement': True
}
u3_info = {
 'id': 'u3',
 'position': [0, -3.5],  # Back
 'height': 1,
 'rotation': 180,
 'outline': [
 pm.Vector2(-1, -0.5),
 pm.Vector2(-1, 0.5),
 pm.Vector2(1, 0.5),
 pm.Vector2(1, -0.5)
 ],
 'visible': True,
 'visible_measurement': True
}

# Infrared sensors for detecting nearby obstacles or target blocks
i0_info = {
    'id': 'i0',
    'position': [1.5, 3],  # Front-left
    'height': 1,
    'rotation': 45,
    'fov': 60,
    'threshold': 0.7,
    'error': 0.05,
    'bias': 0.1,
    'color': (127, 127, 127),
    'visible': True,
    'visible_measurement': True
}
i1_info = {
    'id': 'i1',
    'position': [-1.5, 3],  # Front-right
    'height': 1,
    'rotation': -45,
    'fov': 60,
    'threshold': 0.7,
    'error': 0.05,
    'bias': 0.1,
    'color': (127, 127, 127),
    'visible': True,
    'visible_measurement': True
}

# Gyroscope (for heading/direction information)
g0_info = {
    'id': 'g0',
    'position': [0, 0],
    'rotation': 0,
    'error': 0.02,
    'bias': 0.1,
    'visible': False
}

# Define sensor information
lidar_info = {
    'id': 'li',
    'position': (0, 0),  # Sensor's starting position in the environment
    'rotation': 0,  # Initial orientation (0 degrees)
    'visible': True,
    'height': 2,  # Height of the sensor from the ground
    'color': (0, 0, 255),
    'max_range': 500,  # Maximum detection range
    'error': 0.05,  # 5% error in distance measurements
}



# sensors = {
#     'u0': Ultrasonic(u0_info),
#     'u1': Ultrasonic(u1_info),
#     'u2': Ultrasonic(u2_info),
#     'u3': Ultrasonic(u3_info),
#     'i0': Infrared(i0_info),
#     'i1': Infrared(i1_info),
#     'g0': Gyroscope(g0_info)
# }

sensors = {
    'u0': Ultrasonic(u0_info),
    'li': Lidar(lidar_info)
}

### TESTING AND DEBUG SETTINGS ###
simulate_list = ["li", "u0"]
