import pygame
import config as CONFIG
from lidar import Lidar  # Adjust the import path as needed
from maze import Maze
from block import Block
from robot import Robot
import utilities

# Initialize Pygame (required for using Lidar in the simulation)
pygame.init()

# Mock configurations and environment setup for testing
CONFIG.ppi = 20  # Pixels per inch for display scaling
CONFIG.block_size = 1  # Example block size in inches
CONFIG.border_pixels = 10  # Border offset for drawing
CONFIG.frame_rate = 60
CONFIG.str_encoding = 'ascii'
CONFIG.round_digits = 2

# Create a mock environment with a block and maze
MAZE = Maze()
MAZE.import_walls()
MAZE.generate_floor()
BLOCK = Block()

# Load the Robot and add Lidar sensor
ROBOT = Robot()
lidar_info = {
    'id': 'l0',
    'position': (0, 0),  # Lidar sensor's position on the robot
    'rotation': 0,  # Orientation of the sensor
    'visible': True,
    'height': 2,  # Height from the ground
    'color': (0, 0, 255),
    'max_range': 500,  # Maximum range of the Lidar in inches
    'error': 0.05,  # Error in measurements
}
lidar_sensor = Lidar(lidar_info)

# Environment dictionary for simulation
environment = {
    'BLOCK': BLOCK,
    'MAZE': MAZE,
    'ROBOT': ROBOT
}

# Simulate a Lidar measurement and print results
print("Starting Lidar test...")

# Simulate the sensor and retrieve the distances for each of its rays
lidar_readings = lidar_sensor.simulate(0, environment)

# Print the distances for each of the 8 rays
for i, distance in enumerate(lidar_readings):
    angle = (360 / lidar_sensor.num_rays) * i
    print(f"Ray {i} at angle {angle:.1f} degrees: {distance:.2f} inches")

# Quit Pygame as we are done with the test
pygame.quit()
print("Lidar test completed.")
