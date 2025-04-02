# AUV Mission Control Package

## Overview

The `auv_mission_control` package provides high-level mission planning and execution capabilities for the AUV. It manages the sequence of tasks to be performed during a mission, coordinates with other subsystems (navigation, vision, control), and handles error recovery. The package implements a task-based mission architecture where complex missions are broken down into executable tasks with defined actions.

## Components

### Mission Control (`main.py`)

The core component that orchestrates mission execution:

- Loads and interprets mission plans
- Sequences tasks according to the mission
- Monitors task execution and handles failures
- Manages mission state and progress
- Implements fallback strategies

### Task Execution (`perform_task.py`)

Manages the execution of individual tasks:

- Interprets task actions from configuration
- Coordinates with the navigator for movement
- Interfaces with vision for object detection
- Manages actuator control for interaction tasks
- Provides task success/failure feedback

### AUV Abstraction (`scam_matsya.py`)

Provides a high-level interface to the AUV capabilities:

- Wraps navigation and movement primitives
- Manages task-specific sensing and detection
- Controls actuators (torpedoes, markers, grippers)
- Handles alignment and scanning behaviors
- Monitors vehicle state during task execution

### Environment Map (`map.py`)

Maintains a representation of the mission environment:

- Stores locations of known tasks and objects
- Updates positions based on detections
- Provides spatial references for navigation
- Supports relative positioning between tasks

### Configuration Management (`constants.py`, `config.py`)

Handles loading and management of mission configurations:

- Parses JSON and YAML configuration files
- Provides access to mission plans
- Manages task and action definitions
- Supports dynamic reconfiguration

## Mission Structure

Missions are defined as a sequence of tasks with primary and backup options:

```json
{
    "plan": [
        {
            "main": "Task1",
            "backup": "BackupTask1"
        },
        {
            "main": "Task2",
            "backup": "BackupTask2"
        }
    ]
}
```

Each task consists of a series of actions:

```json
{
    "task_name": "ExampleTask",
    "task_timeout": 120,
    "actions": [
        {
            "name": "reach",
            "reach_timeout": 30,
            "reach_pos_tolerance": 10,
            "reach_angle_tolerance": 5
        },
        {
            "name": "align_continuous",
            "align_xy_tolerance": 10,
            "align_z_tolerance": 10,
            "align_angle_tolerance": 5,
            "align_displacement": {
                "x": -100,
                "y": 0,
                "z": 0,
                "roll": 0,
                "pitch": 0,
                "yaw": 0
            },
            "hard_z_value": 100,
            "scan_type": "vertical_spiral",
            "timeout": 60
        }
    ]
}
```

## Action Types

The mission control system supports various action types:

1. **reach**: Navigate to a specific task location
2. **navigate**: Move to a position relative to current location
3. **raw_navigate**: Direct movement command with specific tolerances
4. **align_continuous**: Continuously align with a detected object
5. **shoot_torpedo**: Fire torpedo at specified target
6. **drop_marker**: Drop marker at current position
7. **go_to_pinger**: Navigate to acoustic pinger source

## ROS2 Interface

### Published Topics

- `/mission_control/current_service_id` (`auv_msgs/CurrentNavId`): Current navigation service ID
- `/feedback/input` (`std_msgs/String`): Feedback from mission execution

### Subscribed Topics

- `/localization/pose` (`auv_msgs/AuvState`): Current vehicle state
- `/vision/detection` (`auv_msgs/VisionSetpoints`): Vision detection results
- `/navigator/status` (`auv_msgs/NavStatus`): Navigation status updates

### Services Used

- `/navigator/setpoint_service` (`auv_msgs/SetpointService`): Point-to-point navigation
- `/navigator/align_service` (`auv_msgs/AlignService`): Alignment with objects
- `/navigator/scan_service` (`auv_msgs/ScanService`): Scanning patterns
- `/vision/set_detector` (`auv_msgs/SetDetector`): Configure vision detector
- `/actuators/torpedo` (`auv_msgs/TorpedoState`): Control torpedo launcher
- `/actuators/gripper` (`auv_msgs/GripperState`): Control gripper
- `/actuators/marker_dropper` (`auv_msgs/MarkerDropperState`): Control marker dropper

### Action Clients

- `/navigator` (`auv_msgs/action/Interrupt`): Navigator action interface

## Configuration Files

The mission control system uses several configuration files:

- `map.json`: Defines task locations in the environment
- `mission.json`: Defines the mission plan with tasks
- `tasks.json`: Defines task details and actions

### Map Configuration

```json
{
    "map": [
        {
            "task": "Buoy",
            "pose": {
                "x": 0,
                "y": 0,
                "z": 30,
                "roll": 0,
                "pitch": 0,
                "yaw": 0
            }
        },
        {
            "task": "Octagon",
            "pose": {
                "x": 50,
                "y": 20,
                "z": 40,
                "roll": 0,
                "pitch": 0,
                "yaw": 0
            }
        }
    ]
}
```

## Usage

### Prerequisites

- ROS2 environment set up
- `auv_msgs` package installed
- `auv_navigator` package running
- `auv_vision` package running
- `auv_localization` package running

### Starting the Mission Control

```bash
ros2 run auv_mission_control main
```

### Creating a New Mission

1. Define task locations in `map.json`
2. Define task actions in `tasks.json`
3. Create mission sequence in `mission.json`
4. Launch the mission control node

### Monitoring Mission Execution

Monitor mission progress through:
- ROS2 topic echo for feedback
- RQT graph for system visualization
- Console output from mission control node

## Mission Execution Flow

The mission execution follows this general flow:

1. **Initialization**: Load configuration and establish ROS connections
2. **Mission Start**: Begin with the first task in the sequence
3. **Task Execution**:
   - Navigate to task location
   - Perform task-specific actions
   - Evaluate task success/failure
4. **Task Completion**:
   - On success: Proceed to next task
   - On failure: Attempt backup task if available
5. **Mission Completion**: End mission after all tasks are attempted

## Error Handling

The mission control system implements several error handling strategies:

- **Task Timeouts**: Each task has a configurable timeout
- **Action Timeouts**: Individual actions have timeouts
- **Backup Tasks**: Alternative tasks can be defined for fallback
- **Position Tolerances**: Configurable tolerances for navigation
- **Detection Thresholds**: Configurable thresholds for object detection

## Development

### Adding a New Task

1. Define the task in `tasks.json` with required actions
2. Add task location to `map.json`
3. Include the task in a mission plan in `mission.json`

### Creating a New Action Type

1. Add action type handling in `perform_task.py`
2. Implement corresponding functionality in `scam_matsya.py`
3. Update documentation and configuration examples

## Troubleshooting

### Common Issues

- **Task Not Starting**: Check localization validity
- **Navigation Failure**: Verify position tolerances and timeout values
- **Vision Integration Issues**: Check camera and detector configuration
- **Action Sequence Problems**: Verify task definition in configuration

### Debugging Tools

- Enable verbose logging for detailed execution information
- Monitor ROS topics for state updates and feedback
- Use service calls to test individual components

## License

Apache License 2.0