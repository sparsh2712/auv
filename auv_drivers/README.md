# AUV Drivers Package

## Overview

The `auv_drivers` package provides the interface between the AUV software and hardware components. It handles communication with sensors (DVL, IMU, pressure sensors, cameras) and actuators (thrusters, torpedoes, marker droppers, grippers) through a centralized message-passing architecture.

## Architecture

The drivers package uses a modular architecture with three main components:

1. **Router** (`router.py`): A central message broker that routes messages between components
2. **USB Driver** (`usb.py`): Handles serial communication with the electronics stack
3. **Node** (`node.py`): ROS2 interface that bridges between ROS topics/services and the router

This design provides:
- Clear separation between hardware communication and ROS interfaces
- Centralized management of hardware messages
- Ability to restart components independently
- Support for plug-and-play hardware detection

## Components

### Router

The router acts as a message broker using ZeroMQ (ZMQ) to route messages between components:

- **Message Format**: `[sender, recipient, data_type, data]`
- **Clients**: USB Driver, ROS Node, GUI (if present)
- **Connection**: TCP socket on port 5555

### USB Driver

The USB driver handles serial communication with the electronics stack:

- **Protocol**: Custom binary protocol with message ID, payload length, payload, and checksum
- **Commands**: Thruster control, servo actuation, power management, etc.
- **Telemetry**: Receives sensor data (IMU, pressure) from the electronics stack

### Node

The ROS2 node provides the interface between ROS and the hardware:

- **Publishers**: Sensor data (IMU, DVL, pressure)
- **Subscribers**: Control commands
- **Services**: Hardware control (torpedo, gripper, marker dropper)

### DVL Driver

Dedicated driver for the Doppler Velocity Log (DVL):

- **Connection**: TCP/IP to DVL
- **Data Format**: JSON messages for velocity and position
- **Publishers**: Velocity and orientation data

### Camera Drivers

Drivers for various cameras:

- **GoPro**: Webcam interface via UDP streaming
- **Basler**: Industrial camera interface
- **OAK-D**: Stereo depth camera interface

## ROS2 Interface

### Published Topics

- `/imu/data` (`sensor_msgs/Imu`): IMU orientation and acceleration data
- `/ps/data` (`auv_msgs/PsData`): Pressure sensor depth data
- `/dvl/velData` (`auv_msgs/DVLVel`): DVL velocity data
- `/dvl/orientData` (`auv_msgs/DVLOrient`): DVL orientation data
- `/front_camera/image_raw` (`sensor_msgs/Image`): Front camera image
- `/bottom_camera/image_raw` (`sensor_msgs/Image`): Bottom camera image
- `/front_camera/oakd_frame` (`auv_msgs/StereoVisionFrame`): OAK-D stereo frame with depth

### Subscribed Topics

- `/controller/pwm` (`auv_msgs/PwmData`): PWM commands for thrusters
- `/controller/setpoint` (`auv_msgs/Pose`): Desired position and orientation

### Services

- `/actuators/torpedo` (`auv_msgs/TorpedoState`): Control torpedo launcher
- `/actuators/gripper` (`auv_msgs/GripperState`): Control gripper
- `/actuators/marker_dropper` (`auv_msgs/MarkerDropperState`): Control marker dropper
- `/thruster/kill` (`std_srvs/Empty`): Emergency stop thrusters
- `/thruster/unkill` (`std_srvs/Empty`): Resume thruster operation

## Usage

### Prerequisites

- ROS2 environment set up
- `auv_msgs` package installed
- Python 3.8+ with required dependencies (ZeroMQ, PySerial, OpenCV)
- USB permissions configured for connected devices

### Starting the Drivers

```bash
# Start the complete driver stack
python3 driver_run.py

# Or launch via ROS2
ros2 launch auv_drivers drivers.launch.py
```

### Configuration

Configuration parameters are provided through YAML files:

- **DVL Configuration**: `config/dvl_params.yaml`
- **Camera Configuration**: Set through command-line arguments to the launch file

### Hardware Connection

1. **Electronics Stack**: Connect via USB serial (typically /dev/ttyACM0)
2. **DVL**: Connect via Ethernet (default IP: 192.168.194.95)
3. **Cameras**: Connect via USB (webcam, OAK-D) or Ethernet (Basler)

## Protocol Details

### Electronics Stack Protocol

The AUV uses a custom binary protocol for communication with the electronics stack:

```
[START_BYTE][MSG_ID][LENGTH][PAYLOAD][CHECKSUM][END_BYTE]
```

Where:
- `START_BYTE`: Fixed value 0xF2
- `MSG_ID`: Message identifier (0x01-0x15)
- `LENGTH`: Length of payload in bytes
- `PAYLOAD`: Message data
- `CHECKSUM`: XOR of all previous bytes
- `END_BYTE`: Fixed value 0xEB

### Message Types

- **0x01**: Thruster PWM values
- **0x02**: Individual thruster PWM
- **0x03**: Thruster kill/unkill
- **0x04**: Thruster limit change
- **0x05**: Thruster timeout change
- **0x06**: Servo actuation
- **0x07**: Servo reset
- **0x08**: Servo custom PWM
- **0x09**: IMU data
- **0x0A**: Pressure sensor data
- **0x0B**: AUV state
- **0x0C**: Log statements
- **0x0D**: Get commands
- **0x0E**: Battery warning
- **0x10**: Temperature warning
- **0x11**: Toggle (power, debug, verbose)
- **0x12**: Unix time
- **0x13**: Mission start
- **0x14**: Reset IMU
- **0x15**: Current time request

## Troubleshooting

### Common Issues

- **Serial Connection Failure**: Check USB permissions and cable connections
- **DVL Communication Error**: Verify network configuration and IP address
- **Camera Not Found**: Check USB connections and device permissions
- **Message Parsing Errors**: Check for protocol mismatches or firmware updates

### Debugging Tools

- **Verbose Mode**: Enable with `send_to_router(USB, "command", ["verbose_on"])`
- **Debug Mode**: Enable with `send_to_router(USB, "command", ["debug_on"])`
- **Port Listing**: Get available ports with `send_to_router(USB, "command", ["get_comport"])`
- **Device Power Control**: Toggle power with `send_to_router(USB, "command", ["power_on/off", ["DEVICE_NAME"]])`

### Log Messages

The system logs messages at several levels:
- **Software Driver Logs**: High-level driver status messages
- **Electronics Stack Logs**: Messages from the microcontroller firmware
- **Hardware Warnings**: Battery, temperature, and other critical warnings

## License

Apache License 2.0