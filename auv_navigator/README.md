# AUV Navigator Package

## Overview

The `auv_navigator` package provides movement primitives and path planning capabilities for the AUV. It serves as the interface between high-level mission commands and low-level control setpoints, implementing various navigation strategies to efficiently move the vehicle through the underwater environment. The navigator is designed to handle the complexities of 3D underwater movement while providing a simple interface for mission control.

## Components

### Navigator Server (`navigator_server.py`)

The main component that implements an action server for navigation requests:

- Processes movement goals from clients
- Implements various navigation primitives
- Manages path planning and execution
- Provides feedback on navigation progress
- Handles interruption and preemption of goals

### Navigation Constants (`constants.py`)

Defines constants and parameters for navigation:

- Tolerance values for position and orientation
- Navigation timeouts and wait times
- Scan patterns for object search

## Navigation Primitives

The navigator implements several movement primitives:

### Setpoint Navigation

Direct movement to a target position and orientation:

- **Depth-First Plan**: Moves in a sequence of depth, yaw, horizontal position, and final orientation
- **Position Tolerances**: Configurable tolerances for each dimension
- **Angle Tolerances**: Configurable tolerances for orientation angles

### Alignment

Movement to align with detected objects:

- **Position Alignment**: Moves to align horizontally and vertically with a target
- **Orientation Alignment**: Rotates to face the target
- **Continuous Alignment**: Iteratively adjusts position during approach

### Scanning

Pattern-based movement for object search:

- **Vertical Spiral**: Helical pattern for 3D volume search
- **Circular**: Rotational pattern for panoramic search
- **Up and Down**: Vertical oscillation for column search

## ROS2 Interface

### Action Server

- `/navigator` (`auv_msgs/action/Interrupt`): Main navigation action interface with:
  - **Goal**: Navigation target with type, position, and tolerances
  - **Feedback**: Current status and progress
  - **Result**: Success or failure status

### Subscribed Topics

- `/localization/pose` (`auv_msgs/AuvState`): Current vehicle state
- `/feedback/input` (`std_msgs/String`): External feedback input (for testing)

### Published Topics

- `/controller/setpoint` (`auv_msgs/Pose`): Setpoints for the controller
- `/navigator/status` (`auv_msgs/NavStatus`): Navigation status updates

## Navigation Types

The navigator supports different navigation approaches through the `current_service_type` field:

1. **setpoint**: Direct movement to a specific position and orientation
2. **align**: Movement to align with an object or position
3. **scan**: Execution of search patterns
4. **scan_align**: Combination of scanning and alignment

## Navigation Algorithms

### Depth-First Plan

For point-to-point navigation, the navigator uses a depth-first approach:

1. **Adjust Depth**: First move to the target depth
2. **Set Heading**: Rotate to face toward the target
3. **Horizontal Movement**: Move horizontally to the target location
4. **Final Orientation**: Adjust to the final orientation

This approach minimizes the complexity of simultaneous multi-axis movement and improves stability.

### Scan Patterns

For object search, the navigator implements several scan patterns:

```python
'vertical_spiral': {
    'relative_setpoints_list': [
        [0, 0, 50, 0, 0, 0],   # Up 50cm
        [0, -50, 0, 0, 0, 0],  # Left 50cm
        [0, 0, -100, 0, 0, 0], # Down 100cm
        [0, 100, 0, 0, 0, 0],  # Right 100cm
        [0, 0, 100, 0, 0, 0],  # Up 100cm
        [0, -50, 0, 0, 0, 0],  # Left 50cm
        [0, 0, -50, 0, 0, 0]   # Down 50cm
    ]
}
```

Each scan pattern consists of a sequence of relative setpoints executed in order.

## Usage

### Prerequisites

- ROS2 environment set up
- `auv_msgs` package installed
- Controller system running
- Localization system running

### Starting the Navigator

```bash
ros2 run auv_navigator navigatorServer
```

### Sending Navigation Goals

Navigation goals can be sent through the action interface:

```python
# Create and send a setpoint navigation goal
goal_msg = Interrupt.Goal()
goal_msg.current_service_type = 'setpoint'
goal_msg.setpoint.position.x = 100.0
goal_msg.setpoint.position.y = 0.0
goal_msg.setpoint.position.z = -50.0
goal_msg.setpoint.orientation.roll = 0.0
goal_msg.setpoint.orientation.pitch = 0.0
goal_msg.setpoint.orientation.yaw = 90.0
goal_msg.pos_tolerance = 10.0
goal_msg.angle_tolerance = 5.0
self._action_client.send_goal_async(goal_msg)
```

### Executing Scan Patterns

```python
# Execute a vertical spiral scan
goal_msg = Interrupt.Goal()
goal_msg.current_service_type = 'scan'
goal_msg.scan_type = 'vertical_spiral'
self._action_client.send_goal_async(goal_msg)
```

### Monitoring Navigation Progress

```bash
# Monitor navigation status
ros2 topic echo /navigator/status
```

## Configuration

### Navigation Parameters

Navigation parameters are defined in `constants.py`:

```python
xy_tolerance = 20.0        # Horizontal position tolerance (cm)
z_tolerance = 30.0         # Vertical position tolerance (cm)
roll_tolerance = 5.0       # Roll angle tolerance (degrees)
pitch_tolerance = 5.0      # Pitch angle tolerance (degrees)
yaw_tolerance = 5.0        # Yaw angle tolerance (degrees)
navigate_time = 2          # Wait time after reaching setpoint (seconds)
scan_wait_time = 2         # Wait time between scan setpoints (seconds)
```

### Scan Patterns

Scan patterns are defined as sequences of relative movements:

```python
scans = {
    'pattern_name': {
        'relative_setpoints_list': [
            [x, y, z, roll, pitch, yaw],
            ...
        ]
    }
}
```

## Advanced Features

### Goal Preemption

The navigator supports preemption of ongoing goals:

- New goals automatically abort previous ones
- Aborted goals return failed status
- Current progress is reported in feedback

### Navigation Timeouts

Timeouts can be specified for navigation operations:

- Overall goal timeout
- Per-setpoint timeout
- Configurable wait times after setpoint achievement

### Feedback Mechanism

The navigator provides feedback during execution:

- Current position relative to target
- Navigation phase (depth, yaw, horizontal, orientation)
- Success/failure status for completed goals

## Troubleshooting

### Common Issues

- **Oscillation**: Decrease controller gains or increase tolerances
- **Slow Navigation**: Adjust timeouts and wait times
- **Path Deviations**: Check localization accuracy and update rates
- **Goal Aborts**: Verify goal parameter validity and sensor availability

### Debugging Tips

- Set larger tolerances for initial testing
- Use simple setpoint navigation before complex patterns
- Verify localization data quality
- Check controller response to setpoints

## License

Apache License 2.0