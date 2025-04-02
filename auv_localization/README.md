# AUV Localization Package

## Overview

The `auv_localization` package provides state estimation for the AUV, fusing data from multiple sensors to determine the vehicle's position, orientation, and velocity in 3D space. It combines inputs from the DVL (Doppler Velocity Log), IMU (Inertial Measurement Unit), and pressure sensor to produce a reliable estimate of the vehicle's state.

## Components

### Localization Node (`localization.py`)

The main localization node integrates sensor data and publishes the estimated AUV state:

- **Sensor Fusion**: Combines measurements from multiple sensors with varying update rates
- **State Estimation**: Calculates position, velocity, orientation, and angular velocity
- **Coordinate Transformation**: Handles transformations between different reference frames
- **Fault Tolerance**: Handles sensor dropouts and provides fallback mechanisms

### IMU Conversion Node (`imu.py`)

Processes raw IMU data to extract orientation information:

- **Quaternion Processing**: Converts quaternion orientation to Euler angles
- **Frame Transformation**: Transforms IMU readings to the vehicle's reference frame
- **Drift Compensation**: Helps mitigate IMU drift issues

## Sensor Integration

The localization system incorporates data from:

1. **DVL (Doppler Velocity Log)**:
   - Provides velocity measurements relative to the seabed
   - Optionally provides position estimates through dead reckoning
   - Provides altitude measurements from the seabed

2. **IMU (Inertial Measurement Unit)**:
   - Provides orientation in quaternion format
   - Provides angular velocity measurements
   - Provides linear acceleration measurements

3. **Pressure Sensor**:
   - Provides depth measurements based on hydrostatic pressure
   - Offers stable Z-axis position information

## ROS2 Interface

### Published Topics

- `/localization/pose` (`auv_msgs/AuvState`): Complete state estimate of the vehicle including:
  - Position (x, y, z)
  - Velocity (x, y, z)
  - Acceleration (x, y, z)
  - Orientation (roll, pitch, yaw)
  - Angular velocity (x, y, z)
  - Height (altitude above seabed)
  - Depth (depth below water surface)

- `/imu/degree` (`auv_msgs/Orientation`): Processed IMU orientation in Euler angles

### Subscribed Topics

- `/imu/data` (`sensor_msgs/Imu`): Raw IMU data
- `/dvl/velData` (`auv_msgs/DVLVel`): DVL velocity data
- `/dvl/orientData` (`auv_msgs/DVLOrient`): DVL orientation data
- `/ps/data` (`auv_msgs/PsData`): Pressure sensor depth data

### Services

- `/localization/reset_service` (`std_srvs/Empty`): Reset localization estimate (typically used at mission start or when new reference points are established)

### Parameters

- **Sensor Status**:
  - `imu_status`: Whether to use IMU data (boolean)
  - `dvl_status`: Whether to use DVL data (boolean)
  - `ps_status`: Whether to use pressure sensor data (boolean)
  - `dvl_vel_integrate`: Whether to use DVL velocity for position integration (boolean)

- **Sensor Orientation**:
  - `imu_orientation`: IMU mounting orientation (roll, pitch, yaw)
  - `dvl_orientation`: DVL mounting orientation (roll, pitch, yaw)

## Algorithm Details

### Sensor Fusion Approach

The localization uses a complementary filter approach:

1. **DVL Integration**: When available, DVL velocity is integrated to estimate position change
2. **IMU Orientation**: IMU provides orientation and helps transform velocities
3. **Pressure Depth**: Pressure sensor provides direct depth measurement
4. **Position Reset**: The system allows for position resets to handle drift

### Position Estimation

Position is estimated through:
- Direct integration of DVL velocity when available
- Dead reckoning using IMU when DVL is unavailable
- Absolute depth from pressure sensor

### Orientation Estimation

Orientation is estimated using:
- Primary: IMU quaternion data converted to Euler angles
- Secondary: DVL orientation data when IMU is unavailable

## Usage

### Prerequisites

- ROS2 environment set up
- `auv_msgs` package installed
- Working sensor drivers (IMU, DVL, pressure sensor)

### Starting the Localization System

```bash
ros2 launch auv_localization localization.launch.py
```

### Visualization

To visualize the vehicle's estimated position and orientation:

```bash
# Plot position over time
ros2 run rqt_plot rqt_plot /localization/pose/position/x /localization/pose/position/y /localization/pose/position/z

# Plot orientation over time
ros2 run rqt_plot rqt_plot /localization/pose/orientation/roll /localization/pose/orientation/pitch /localization/pose/orientation/yaw
```

### Configuration

Configure the localization system through YAML files:

```yaml
# Example config in localization_constants.yaml
localization:
  ros__parameters:
    imu_orientation:
      roll: 0.0
      pitch: 0.0
      yaw: 180.0
    dvl_orientation:
      roll: 0.0
      pitch: 0.0
      yaw: 0.0
    imu_status: true
    dvl_status: true
    ps_status: true
    dvl_vel_integrate: true
```

## Coordinate Systems

The localization system uses the following coordinate conventions:

1. **Body Frame**:
   - X: Forward
   - Y: Right
   - Z: Down
   - Roll: Rotation around X
   - Pitch: Rotation around Y
   - Yaw: Rotation around Z

2. **World Frame**:
   - X: East
   - Y: North
   - Z: Up
   - Origin: Mission start point (resets available)

## Performance Considerations

- **Update Rate**: Localization updates at approximately 10Hz
- **Position Accuracy**: Dependent on DVL quality, typically sub-meter
- **Orientation Accuracy**: Dependent on IMU quality, typically less than 2 degrees
- **Drift**: Position will drift over time, especially without DVL bottom-lock

## Troubleshooting

### Common Issues

- **Position Drift**: Reset localization or improve DVL bottom-lock
- **Orientation Jumps**: Check IMU mounting and calibration
- **Z-Axis Instability**: Verify pressure sensor calibration

### Debugging

- Set the relevant sensor status parameters to false to isolate issues
- Check sensor measurements directly by inspecting raw topics
- Use the reset service to re-establish a known position

## License

Apache License 2.0