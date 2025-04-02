# AUV Controller Package

## Overview

The `auv_controller` package implements the motion control system for the Autonomous Underwater Vehicle (AUV). It handles the transformation of desired vehicle movements into individual thruster commands, implementing a hierarchical control approach with PID controllers for each degree of freedom and a sophisticated thruster allocation matrix.

## Components

### PID Control (`pid_control.py`)
The PID controller module implements closed-loop control for six degrees of freedom (DoF):
- Surge (forward/backward motion)
- Sway (left/right motion)
- Heave (up/down motion)
- Roll (rotation around forward axis)
- Pitch (rotation around lateral axis)
- Yaw (rotation around vertical axis)

This module computes the desired forces and torques to achieve the setpoint position and orientation.

### Thruster Allocator (`allocator.py`)
The thruster allocator transforms the desired forces and torques computed by the PID controllers into individual thruster commands using a Control Effectiveness Matrix (CEM). Key features include:
- Handling of thruster saturation
- Optimization of thruster allocation for efficiency
- Support for different thruster configurations
- Management of thruster deadbands and non-linearities

### Force-to-PWM Mapping
Converts calculated thrust values to PWM signals sent to the motor controllers, using:
- Calibrated thrust curves
- Backlash compensation
- Direction-dependent scaling

### Controller Node (`controller.py`)
The main ROS2 node that:
- Subscribes to setpoint and sensor topics
- Manages the control loop timing
- Publishes thruster commands
- Handles parameter updates

## ROS2 Interface

### Published Topics
- `/controller/pwm` (`auv_msgs/PwmData`): PWM values for each thruster
- `/controller/thruster_forces` (`auv_msgs/ThrusterForces`): Calculated forces for each thruster
- `/controller/global_forces` (`auv_msgs/GlobalForces`): Desired forces and torques in the global frame

### Subscribed Topics
- `/controller/setpoint` (`auv_msgs/Pose`): Desired position and orientation
- `/localization/pose` (`auv_msgs/AuvState`): Current vehicle state

### Services
- `/controller/pwm_cap` (`auv_msgs/PwmCap`): Set maximum PWM value for thrusters

### Parameters
- **PID Constants**: Tuning parameters for each DoF controller
  - `Kp`: Proportional gain
  - `Ki`: Integral gain
  - `Kd`: Derivative gain
  - `maxFront`: Maximum positive force/torque
  - `maxBack`: Maximum negative force/torque

- **Thruster Configuration**:
  - `num_thrust`: Number of thrusters
  - `COM`: Center of mass coordinates
  - `thrust_pose`: Thruster positions relative to COM
  - `thrust_orien`: Thruster orientations
  - `max_front_force`: Maximum forward thrust
  - `max_back_force`: Maximum backward thrust
  - `max_pwm`: Maximum PWM value
  - `min_pwm`: Minimum PWM value
  - `zero_pwm`: PWM value for zero thrust

- **Control Effectiveness Matrix (CEM) Configuration**:
  - `power_factors`: Scaling factors for each thruster
  - `backward_ratios`: Asymmetry compensation for thrusters

## Usage

### Prerequisites
- ROS2 environment set up
- `auv_msgs` package installed

### Starting the Controller
```bash
ros2 launch auv_controller controller.launch.py
```

### Configuration
1. Configure thruster positions and orientations in `config/thruster_positions.yaml`
2. Tune PID parameters in `config/pid_constants.yaml`
3. Set the Control Effectiveness Matrix parameters in respective CEM YAML files

### Tuning
The controller can be tuned using the `allocator_tuning.py` node, which provides:
- Systematic testing of individual thrusters
- Verification of thruster allocation
- Force application testing

```bash
ros2 run auv_controller allocatorTuning
```

## Algorithm Details

### Thruster Allocation

The thruster allocation uses a pseudo-inverse of the Control Effectiveness Matrix (CEM) to convert desired body forces/torques to individual thruster forces. The CEM is decomposed into separate matrices for:
- Surge motion
- Sway motion
- Heave motion
- Roll, Pitch, and Yaw (RPY) rotations

The allocation process:
1. Split requested body forces into components
2. Use each specific CEM to compute thruster contributions
3. Adjust for backward motion using ratio factors
4. Cap thrust values to respect thruster limits
5. Convert thrust values to PWM signals

### PID Control

The PID control computes the desired forces and torques to minimize the error between the current and desired states:
1. Compute error in the body frame
2. Apply PID control to each DoF
3. Apply force/torque limits
4. Output desired body forces/torques

## Troubleshooting

### Common Issues
- **Thruster Not Responding**: Check PWM ranges and connections
- **Oscillating Behavior**: Reduce Kp or increase Kd
- **Steady-State Error**: Increase Ki
- **Unexpected Motion**: Verify thruster positions and orientations

### Diagnostics
- Monitor `/controller/global_forces` to verify PID output
- Check `/controller/thruster_forces` to verify allocation
- Inspect `/controller/pwm` to verify PWM mapping

## License
Apache License 2.0