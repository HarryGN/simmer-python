# Autonomous rover

This project focuses on creating an autonomous robot powered by an Arduino and a list of sensors. The robot uses two DC motors for movement RPlidarA1 for balancing and orientation sensing. The goal is to design and build a robot capable of autonomously navigating a known maze.

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Hardware Design](#hardware-design)
- [Control Strategy](#control-strategy)
  - [Obstacle Avoidance](#obstacle-avoidance)
    - [Lidar-based Obstacle Avoidance (Milestone 1)](#lidar-based-obstacle-avoidance-milestone-1)
    - [TOF-based Obstacle Avoidance (Milestone 2)](#tof-based-obstacle-avoidance-milestone-2)
  - [Localization and Navigation](#localization-and-navigation)
    - [Lidar-Based Localization (Milestone 2)](#lidar-based-localization-milestone-2)
- [Real-life Testing](#real-life-testing)
- [Installation](#installation)
- [License](#license)

## Overview
The UniWheel robot is a mechatronic system that balances itself using an IMU sensor and navigates its environment autonomously using a combination of LIDAR, TOF sensors, and other sensing technologies. The robot is controlled by an ESP32 microcontroller, which processes sensor data and makes real-time decisions to maintain balance and avoid obstacles.

## Features
- **Obstacle Avoidance**: The robot is equipped with TOF sensors for obstacle detection and avoidance.
- **Lidar Localization with Orientation**: The robot is equipped with LIDAR localization.
- **Autonomous Navigation**: The robot can navigate through predefined environments, avoiding obstacles and following paths.
- **Real-time Control**: Powered by an Arduino and bluetooth, the robot processes sensor data and adjusts movements in real time.

## Hardware Design
![mmexport1728095839496](https://github.com/user-attachments/assets/3533c752-4f3b-4894-8044-40482368458d)
The robot is powered by an Arduino Mega microcontroller and uses DC motors to control movement. The LIDAR and TOF sensors provide the necessary data for localization and obstacle avoidance.

#### First Iteration Design
![First Iteration](https://github.com/user-attachments/assets/0833e858-9b5e-44f8-abbe-aa0cd2b539af)

#### Overall Design
![Overall Design](https://github.com/user-attachments/assets/5011e34f-6a4d-4bc9-8ffd-2e0424a839de)

#### Design with TOF and LIDAR
![Design with TOF and LIDAR](https://github.com/user-attachments/assets/88bfff5b-a361-4fd6-9bf5-9c0bde1c5fe7)

## Control Strategy

### Obstacle Avoidance
The rover utilizes a multi-stage obstacle avoidance strategy based on sensor data.

#### Lidar-based Obstacle Avoidance (Milestone 1)
In the initial stage, the robot used a LIDAR sensor to measure distances in multiple directions (front, left, and right). The algorithm made decisions based on the distance measurements:
- If the robot was surrounded by obstacles on three sides, it executed a 180° turn.
- For obstacles directly ahead, the robot stopped, reversed, and then turned left or right depending on available space.
- give up on slow and massive data process.

#### TOF-based Obstacle Avoidance (Milestone 2)
The robot was equipped with seven TOF sensors for more precise short-range measurements. These sensors provided data for obstacle detection and helped the robot make decisions in confined spaces:
- If obstacles were detected ahead, the robot stopped, reversed, and turned 90°.
- The robot also adjusted its path to avoid diagonal obstacles and maintained consistent distance from walls for smooth navigation.

### Localization and Navigation

#### Lidar-Based Localization (Milestone 2)
Localization was achieved by comparing LIDAR data with a preloaded map of the environment. The robot's position and orientation were estimated by matching the sensor data with expected readings in a pre-simulated matrix.
- The robot generated a confidence map that visually represented the likelihood of the robot's position based on LIDAR readings.
- Movements were tracked and updated to refine the robot's location continuously.
However, due to delays in data processing, this approach was replaced by a faster ultrasonic wall-following method in Milestone 3.

## Real-life Testing
The robot was tested in real-world environments, where the obstacle avoidance strategies and localization methods were evaluated for accuracy and performance.

![Real-life Run](https://github.com/user-attachments/assets/4255ab42-5a16-46b3-ada2-d54c76698c04)

## Installation
To get started with the UniWheel project, follow these steps:

1. **Clone the repository:**

    ```bash
    git clone https://github.com/HarryGN/simmer-python.git
    ```

2. **Install dependencies:**
   - Python 3.12 or higher
   - Pygame 2.6 or higher
   - Numpy 1.26 or higher
   - Shapely 2.05 or higher

3. **Upload the code to your Arduino Mega**: Use your preferred IDE (e.g., Arduino IDE) to upload the code to the Arduino.

4. **Connect the hardware**: Ensure that the LIDAR, TOF sensors, and battery are connected to the appropriate pins on the Arduino.

5. **Run the simulator**: Use the provided Python scripts to simulate the robot's behavior in various environments then test in real life!

## License
This project is open-source and distributed under the GNU Affero General Public License. See the LICENSE for more details.
