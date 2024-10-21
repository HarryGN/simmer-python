import math
import pygame
import pygame.math as pm
from devices.device import Device
import config as CONFIG
import utilities

class Lidar(Device):
    '''Defines an lidar sensor with 8 rays distributed over 360 degrees.'''

    def __init__(self, info: dict):
        '''Initialization'''

        # Call super initialization
        super().__init__(info['id'], info['position'], info['rotation'], info['visible'])

        # Device type (i.e. "drive", "motor", or "sensor")
        self.d_type = 'sensor'
        self.name = 'lidar'

        # Device outline position
        self.outline = info.get('outline', [
            pm.Vector2(-1, -0.5),
            pm.Vector2(-1, 0.5),
            pm.Vector2(1, 0.5),
            pm.Vector2(1, -0.5)
        ])

        # Device height
        self.height = info.get('height', CONFIG.block_size)

        # Display color
        self.color = info.get('color', (0, 0, 255))

        # Display thickness
        self.outline_thickness = info.get('outline_thickness', 0.25)

        # Display measurement when simulating
        self.visible_measurement = info.get('visible_measurement', True)
        self.visible_measurement_time = info.get('visible_measurement_time', 0.5)    # Measurement time on screen (s)
        self.visible_measurement_buffer = 0

        # Simulation parameters
        self.num_rays = 8                             # Number of rays (8 for 360-degree coverage)
        self.min_range = info.get('min_range', 0)     # Minimum range in inches
        self.max_range = info.get('max_range', 433)   # Maximum range in inches
        self.error_pct = info.get('error', 0.02)      # Percent error (0-1)
        self.reading_bounds = [self.min_range, self.max_range]  # Upper and lower bounds for sensor reading

        # Define initial rays
        self.rays = self._define_rays()
        self.ray_lengths_squared = [self.max_range**2 for _ in self.rays]  # The length of the rays

    def _define_rays(self):
        '''Define the rays used to get the ultrasonic distance.'''

        rays = []
        for ct in range(self.num_rays):
            # Calculate the angle of each ray (45-degree intervals for 8 rays)
            angle_ray = (360 / self.num_rays) * ct
            angle_ray_global = angle_ray + self.rotation_global

            # Calculate the start and end points of each ray
            direction = pygame.math.Vector2(0, self.max_range)
            ray_end = pygame.math.Vector2.rotate(direction, angle_ray_global) + self.position_global

            # Append the calculated rays
            rays.append([self.position_global, ray_end])

        return rays

    def draw_measurement(self, canvas):
        '''Draw ultrasonic sensor rays on the canvas'''

        # If the measurement should be displayed
        if self.visible_measurement_buffer:

            # Graphics options
            thickness = int(CONFIG.ppi * 0.125)
            color = (0, 0, 255)

            # Draw the lines on the canvas
            for ray in self.rays:
                start = [point * CONFIG.ppi + CONFIG.border_pixels for point in ray[0]]
                end = [point * CONFIG.ppi + CONFIG.border_pixels for point in ray[1]]
                pygame.draw.line(canvas, color, start, end, thickness)
            # Decrement the buffer
            self.visible_measurement_buffer -= 1

    def simulate(self, value: float, environment: dict):
        '''
        Simulates the performance of a LiDAR sensor.

        Response data format:
        Single integer that encodes all 8 direction values with one decimal place.
        '''
        MAZE = environment.get("MAZE", False)
        BLOCK = environment.get('BLOCK', False)

        rays = self._define_rays()
        ray_lengths_squared = [self.max_range**2 for _ in rays]

        # Update the measurement display buffer
        self.visible_measurement_buffer = int(self.visible_measurement_time * CONFIG.frame_rate)

        # Check if the sensor is at a height where the block would be seen
        if self._block_visible(BLOCK):
            walls_to_check = BLOCK.block_square + MAZE.reduced_walls
        else:
            walls_to_check = MAZE.reduced_walls

        for ct, ray in enumerate(rays):
            for wall in walls_to_check:
                collision_points = utilities.collision(ray, wall)
                if collision_points:
                    rays[ct][1], ray_lengths_squared[ct] = utilities.closest_fast(
                        self.position_global, collision_points
                    )

        # Update stored variables
        self.rays = rays
        self.ray_lengths_squared = ray_lengths_squared

        # Calculate the distances for all rays with one decimal place precision
        distances = [round(math.sqrt(length), 1) for length in self.ray_lengths_squared]
        distances = [utilities.add_error(dist, self.error_pct, self.reading_bounds) for dist in distances]

        # Convert each distance to an integer with one decimal place (e.g., 12.3 -> 123)
        encoded_distances = [int(dist * 10) for dist in distances]

        # Combine the encoded distances into a single integer
        combined_value = int(''.join(f'{value:03d}' for value in encoded_distances))

        # print(f"Encoded distances: {encoded_distances}")
        # print(f"Combined encoded value: {combined_value}")
        
        # Decode after encoding for testing (in a real scenario, this might be done elsewhere)
        # decoded_distances = decode_combined_value(combined_value)
        # print(f"Decoded distances: {decoded_distances}")

        return combined_value

    def _block_visible(self, BLOCK):
        '''Determines whether the block is visible to an ultrasonic sensor based on its height.'''

        # Get some geometric parameters
        view_angle_z = 180  # For 360-degree detection, assume full view in the z-plane
        h = self.height - BLOCK.height
        vec = pm.Vector2(BLOCK.position.x - self.position_global.x, BLOCK.position.y - self.position_global.y)

        # Calculate distance and angle to block
        d = vec.magnitude()
        th = math.atan(h / d) * 180 / math.pi

        # If angle to block is less than sensor view angle, it's visible
        return th <= view_angle_z
