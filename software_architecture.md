# AUV Software Architecture Overview

## Introduction

This document provides a high-level overview of the software architecture for our Autonomous Underwater Vehicle (AUV) system. The software is built using ROS2 (Robot Operating System 2) and is designed to enable autonomous navigation, perception, and mission execution for underwater robotics applications.

## System Architecture

The AUV software is composed of five primary subsystems that work together to enable autonomous underwater operations:

1. **Controller System** - Manages vehicle motion and stability
2. **Driver System** - Interfaces with hardware components
3. **Localization System** - Determines vehicle position and orientation
4. **Vision System** - Processes visual data to detect objects
5. **Mission Control System** - Executes high-level mission plans
6. **Navigator System** - Provides movement primitives and path planning

## Data Flow

The system operates with the following general data flow:

1. **Sensor Data Acquisition**: The Driver System interfaces with hardware sensors (cameras, DVL, IMU, pressure sensor) to collect raw data
2. **Localization**: Sensor data is processed by the Localization System to estimate vehicle position and orientation
3. **Perception**: The Vision System processes camera data to detect objects of interest
4. **Mission Execution**: The Mission Control System uses localization and perception data to execute tasks
5. **Navigation**: The Navigator System translates high-level movement commands into trajectories
6. **Control**: The Controller System converts desired trajectories into thruster commands

## Subsystems

### Controller System (`auv_controller`)

The Controller System manages the vehicle's motion and stability using a hierarchical control approach:

- **PID Control**: Implements PID controllers for six degrees of freedom (surge, sway, heave, roll, pitch, yaw)
- **Thruster Allocation**: Converts desired forces and torques into individual thruster commands
- **Force-to-PWM Mapping**: Maps calculated thrust values to PWM signals for motor controllers

### Driver System (`auv_drivers`)

The Driver System acts as the interface between software and hardware:

- **Message Router**: Central communication hub for hardware messages
- **Sensor Drivers**: Interfaces for DVL, IMU, pressure sensor, and cameras
- **Actuator Control**: Interfaces for thrusters, torpedo launchers, marker droppers, and grippers

### Localization System (`auv_localization`)

The Localization System fuses data from multiple sensors to estimate the vehicle's position and orientation:

- **Sensor Fusion**: Combines DVL, IMU, and pressure sensor data
- **State Estimation**: Calculates position, velocity, and orientation
- **Coordinate Transformations**: Handles conversions between different reference frames

### Vision System (`auv_vision`)

The Vision System processes camera data to detect and locate objects:

- **Image Processing**: Applies filters and preprocessing for underwater images
- **Object Detection**: Uses both classical CV and deep learning approaches (YOLOv5/v8)
- **3D Localization**: Maps 2D detections to 3D world coordinates
- **Task-Specific Detection**: Customized detection for competition elements (buoys, bins, torpedoes)

### Mission Control System (`auv_mission_control`)

The Mission Control System manages high-level planning and execution:

- **Task Scheduling**: Sequences tasks according to mission requirements
- **Task Execution**: Monitors and manages the execution of individual tasks
- **Error Recovery**: Handles failures and executes recovery behaviors
- **Map Management**: Maintains a map of the environment and task locations

### Navigator System (`auv_navigator`)

The Navigator System provides movement capabilities:

- **Action Server**: ROS2 action interface for mission control to request movements
- **Path Planning**: Generates motion paths for different navigation objectives
- **Movement Primitives**: Implements scanning patterns, alignment, and point-to-point movement
- **Setpoint Management**: Translates desired positions into controller setpoints

## Communication Architecture

The system uses a hybrid communication approach:

- **ROS2 Topics**: For high-frequency data streaming (sensor data, control commands)
- **ROS2 Services**: For request-response patterns (configuration, mode changes)
- **ROS2 Actions**: For long-running tasks with feedback (navigation, mission execution)
- **ZMQ Messaging**: For hardware communication and inter-process messaging

## Configuration Management

Configuration is managed through several mechanisms:

- **YAML Files**: For ROS2 parameters and general system configuration
- **JSON Files**: For mission plans and task definitions
- **Constants Files**: For algorithm-specific parameters