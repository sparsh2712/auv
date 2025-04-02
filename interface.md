# AUV Interface Documentation

This document details the interfaces between the major components of the AUV software stack, including topics, services, actions, and message types.

## System Interface Diagram

The following diagram illustrates the major interfaces between components:

```
┌─────────────────┐   setpoint   ┌─────────────────┐   pwm    ┌─────────────────┐
│                 │─────────────▶│                 │─────────▶│                 │
│ Mission Control │              │    Navigator    │          │   Controller    │
│                 │◀─────────────│                 │          │                 │
└─────────────────┘   status     └─────────────────┘          └─────────────────┘
        │                               │                             │
        │ tasks                         │ setpoint                    │ forces
        │                               │                             │
        ▼                               ▼                             ▼
┌─────────────────┐   detection  ┌─────────────────┐  sensor  ┌─────────────────┐
│                 │─────────────▶│                 │◀─────────│                 │
│     Vision      │              │  Localization   │          │     Drivers     │
│                 │◀─────────────│                 │─────────▶│                 │
└─────────────────┘   pose       └─────────────────┘  cmds    └─────────────────┘
```

## ROS Topics

### Mission Control Topics

| Topic Name | Message Type | Publisher | Subscribers | Description |
|------------|--------------|-----------|-------------|-------------|
| `/mission_control/current_service_id` | `auv_msgs/CurrentNavId` | Mission Control | Navigator | Current navigation service ID |
| `/feedback/input` | `std_msgs/String` | Mission Control | Navigator | Feedback for testing and debugging |

### Navigator Topics

| Topic Name | Message Type | Publisher | Subscribers | Description |
|------------|--------------|-----------|-------------|-------------|
| `/controller/setpoint` | `auv_msgs/Pose` | Navigator | Controller | Target position and orientation |
| `/navigator/status` | `auv_msgs/NavStatus` | Navigator | Mission Control | Navigation status updates |

### Controller Topics

| Topic Name | Message Type | Publisher | Subscribers | Description |
|------------|--------------|-----------|-------------|-------------|
| `/controller/pwm` | `auv_msgs/PwmData` | Controller | Drivers | PWM values for thrusters |
| `/controller/thruster_forces` | `auv_msgs/ThrusterForces` | Controller | Diagnostics | Forces applied to each thruster |
| `/controller/global_forces` | `auv_msgs/GlobalForces` | Controller | Diagnostics | Desired forces and torques |

### Localization Topics

| Topic Name | Message Type | Publisher | Subscribers | Description |
|------------|--------------|-----------|-------------|-------------|
| `/localization/pose` | `auv_msgs/AuvState` | Localization | Multiple | Complete vehicle state estimate |
| `/imu/degree` | `auv_msgs/Orientation` | Localization | Diagnostics | Processed IMU orientation angles |

### Vision Topics

| Topic Name | Message Type | Publisher | Subscribers | Description |
|------------|--------------|-----------|-------------|-------------|
| `/vision/detection` | `auv_msgs/VisionSetpoints` | Vision | Mission Control | Detected objects and their positions |
| `/camera/bbox_image` | `sensor_msgs/Image` | Vision | Visualization | Debug image with bounding boxes |
| `/vision/oakd_bbox` | `auv_msgs/BBoxes` | Vision | Internal | Bounding boxes from OAK-D camera |
| `/vision/rel_position` | `auv_msgs/VisionSetpoints` | OAK-D | Vision | Relative positions from depth camera |

### Driver Topics

| Topic Name | Message Type | Publisher | Subscribers | Description |
|------------|--------------|-----------|-------------|-------------|
| `/imu/data` | `sensor_msgs/Imu` | Drivers | Localization | Raw IMU data |
| `/ps/data` | `auv_msgs/PsData` | Drivers | Localization | Pressure sensor data |
| `/dvl/velData` | `auv_msgs/DVLVel` | Drivers | Localization | DVL velocity data |
| `/dvl/orientData` | `auv_msgs/DVLOrient` | Drivers | Localization | DVL orientation data |
| `/front_camera/image_raw` | `sensor_msgs/Image` | Drivers | Vision | Front camera feed |
| `/bottom_camera/image_raw` | `sensor_msgs/Image` | Drivers | Vision | Bottom camera feed |
| `/front_camera/oakd_frame` | `auv_msgs/StereoVisionFrame` | Drivers | Vision | OAK-D stereo frame with depth |

## ROS Services

### Mission Control Services

None directly provided, uses services from other components.

### Driver Services

| Service Name | Service Type | Provider | Description |
|--------------|--------------|----------|-------------|
| `/actuators/torpedo` | `auv_msgs/TorpedoState` | Drivers | Control torpedo launcher |
| `/actuators/gripper` | `auv_msgs/GripperState` | Drivers | Control gripper |
| `/actuators/marker_dropper` | `auv_msgs/MarkerDropperState` | Drivers | Control marker dropper |
| `/thruster/kill` | `std_srvs/Empty` | Drivers | Emergency stop thrusters |
| `/thruster/unkill` | `std_srvs/Empty` | Drivers | Resume thruster operation |

### Vision Services

| Service Name | Service Type | Provider | Description |
|--------------|--------------|----------|-------------|
| `/vision/model` | `auv_msgs/VisionModel` | Vision | Set current task/model for detection |
| `/vision/toggle_camera` | `auv_msgs/SetDetector` | Vision | Switch between cameras |
| `/vision/set_detector` | `auv_msgs/SetDetector` | Vision | Configure detector parameters |

### Localization Services

| Service Name | Service Type | Provider | Description |
|--------------|--------------|----------|-------------|
| `/localization/reset_service` | `std_srvs/Empty` | Localization | Reset localization estimate |

### Controller Services

| Service Name | Service Type | Provider | Description |
|--------------|--------------|----------|-------------|
| `/controller/pwm_cap` | `auv_msgs/PwmCap` | Controller | Set maximum PWM value for thrusters |

## ROS Actions

### Navigator Actions

| Action Name | Action Type | Provider | Description |
|-------------|-------------|----------|-------------|
| `/navigator` | `auv_msgs/action/Interrupt` | Navigator | Main navigation action interface |

## Message Types

### State and Pose Messages

| Message Type | Description | Key Fields |
|--------------|-------------|------------|
| `auv_msgs/AuvState` | Complete vehicle state | position, velocity, acceleration, orientation, angular_velocity, height, depth |
| `auv_msgs/Pose` | Position and orientation | position, orientation |
| `auv_msgs/Orientation` | Euler angles | roll, pitch, yaw |

### Control Messages

| Message Type | Description | Key Fields |
|--------------|-------------|------------|
| `auv_msgs/PwmData` | Thruster PWM values | surge_front_left, surge_front_right, etc. |
| `auv_msgs/ThrusterForces` | Thruster force values | surge_top, surge_left, etc. |
| `auv_msgs/GlobalForces` | Global forces and torques | force, torque |

### Sensor Messages

| Message Type | Description | Key Fields |
|--------------|-------------|------------|
| `auv_msgs/DVLVel` | DVL velocity data | velocity, fom, altitude |
| `auv_msgs/DVLOrient` | DVL orientation data | position, roll, pitch, yaw |
| `auv_msgs/DVLBeam` | DVL beam data | id, velocity, distance, rssi, nsd, valid |
| `auv_msgs/PsData` | Pressure sensor data | depth |
| `sensor_msgs/Imu` | IMU data | orientation, angular_velocity, linear_acceleration |

### Vision Messages

| Message Type | Description | Key Fields |
|--------------|-------------|------------|
| `auv_msgs/VisionSetpoints` | Vision detection results | objects, tasks |
| `auv_msgs/ObjectDetected` | Detected object data | name, position |
| `auv_msgs/TaskDetected` | Detected task data | name, position |
| `auv_msgs/BoundingBox` | Detection bounding box | class_id, class_name, xmin, ymin, xmax, ymax, confidence |
| `auv_msgs/BBoxes` | Collection of bounding boxes | bboxes |
| `auv_msgs/StereoVisionFrame` | Stereo camera frame | camera_frame, depth_frame |

### Navigation Messages

| Message Type | Description | Key Fields |
|--------------|-------------|------------|
| `auv_msgs/NavStatus` | Navigation status | service_id, status |
| `auv_msgs/CurrentNavId` | Current navigation ID | current_service_id |

## Service Types

| Service Type | Description | Request | Response |
|--------------|-------------|---------|----------|
| `auv_msgs/TorpedoState` | Control torpedo | torpedo_bits | (none) |
| `auv_msgs/GripperState` | Control gripper | gripper_bits | (none) |
| `auv_msgs/MarkerDropperState` | Control marker dropper | marker_dropper_bits | (none) |
| `auv_msgs/SetDetector` | Configure vision | camera | (none) |
| `auv_msgs/VisionModel` | Set vision model | model | (none) |
| `auv_msgs/PwmCap` | Set PWM cap | max_pwm | (none) |
| `std_srvs/Empty` | Empty service | (none) | (none) |

## Action Types

| Action Type | Description | Goal | Feedback | Result |
|-------------|-------------|------|----------|--------|
| `auv_msgs/action/Interrupt` | Navigation action | current_service_type, scan_type, setpoint, pos_tolerance, angle_tolerance, xy_tolerance, z_tolerance | setpoint_detect, task_name, action | success |

## Parameters

### Controller Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `/controller/pid_constants/surge/Kp` | float | 0.5 | Proportional gain for surge |
| `/controller/pid_constants/surge/Ki` | float | 0.0 | Integral gain for surge |
| `/controller/pid_constants/surge/Kd` | float | 0.7 | Derivative gain for surge |
| `/controller/pid_constants/surge/maxBack` | float | -70.0 | Maximum backward force for surge |
| `/controller/pid_constants/surge/maxFront` | float | 70.0 | Maximum forward force for surge |

### Localization Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `/localization/imu_status` | bool | true | Whether to use IMU data |
| `/localization/dvl_status` | bool | true | Whether to use DVL data |
| `/localization/ps_status` | bool | true | Whether to use pressure sensor data |
| `/localization/dvl_vel_integrate` | bool | true | Whether to integrate DVL velocity |
| `/localization/imu_orientation/roll` | float | 0.0 | IMU mounting roll offset |
| `/localization/imu_orientation/pitch` | float | 0.0 | IMU mounting pitch offset |
| `/localization/imu_orientation/yaw` | float | 180.0 | IMU mounting yaw offset |

### Vision Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `/vision/front_camera_config/img_height` | int | 720 | Front camera image height |
| `/vision/front_camera_config/img_width` | int | 1280 | Front camera image width |
| `/vision/dvl_to_camera_vector/front/x` | float | 0.0 | X offset from DVL to front camera |
| `/vision/dvl_to_camera_vector/front/y` | float | 0.0 | Y offset from DVL to front camera |
| `/vision/dvl_to_camera_vector/front/z` | float | 0.0 | Z offset from DVL to front camera |

## Data Flow Examples

### Mission Execution Flow

1. Mission Control loads task from configuration
2. Mission Control sends navigation goal to Navigator (action)
3. Navigator generates trajectory and sends setpoints to Controller (topic)
4. Controller computes forces and sends PWM values to Drivers (topic)
5. Drivers send commands to hardware thrusters
6. Localization processes sensor data and publishes vehicle state (topic)
7. Vision processes camera data and detects objects (topic)
8. Mission Control monitors vision and localization data to control task execution
9. Upon task completion, Mission Control moves to next task

### Object Detection and Alignment Flow

1. Mission Control sends vision model request to Vision (service)
2. Vision configures detector for specific task/object
3. Drivers publish camera feed to Vision (topic)
4. Vision processes images and publishes detections (topic)
5. Mission Control receives detection and computes alignment position
6. Mission Control sends alignment goal to Navigator (action)
7. Navigator generates trajectory and sends setpoints to Controller (topic)
8. Controller computes forces and sends PWM values to Drivers (topic)
9. Drivers send commands to hardware thrusters
10. Process continues iteratively until alignment achieved

### Torpedo Shooting Flow

1. Mission Control sends vision model request for torpedo target (service)
2. Vision detects target and publishes position (topic)
3. Mission Control sends alignment goal to Navigator (action)
4. Navigator aligns vehicle with target
5. Mission Control checks alignment quality and sends torpedo command (service)
6. Drivers actuate torpedo launcher
7. Mission Control verifies success and moves to next task

## Interface Changes and Versioning

When making changes to interfaces:

1. **Message Extensions**: Add new fields at the end of messages to maintain backward compatibility
2. **Service Changes**: Create new service types rather than modifying existing ones
3. **Topic Renaming**: Use remapping when topics need to be renamed
4. **Version Tracking**: Include version numbers in package.xml and track interface changes in CHANGELOG.md

## Interface Testing

### Testing Tools

```bash
# Test topic publishing
ros2 topic pub /topic_name message_type "data" --once

# Test topic monitoring
ros2 topic echo /topic_name

# Test service call
ros2 service call /service_name service_type "request_data"

# Test action
ros2 action send_goal /action_name action_type "goal_data"
```

### Topic Diagnostics

```bash
# Check topic statistics
ros2 topic info /topic_name -v

# Monitor topic rate
ros2 topic hz /topic_name
```

## Common Interface Patterns

### Publisher-Subscriber

Used for continuous data streams like sensor readings and states:

```python
# Publisher
self.publisher = self.create_publisher(MsgType, 'topic_name', 10)
self.publisher.publish(msg)

# Subscriber
self.subscription = self.create_subscription(
    MsgType,
    'topic_name',
    self.callback_function,
    10)
```

### Service Client-Server

Used for request-response interactions:

```python
# Service Server
self.service = self.create_service(
    SrvType, 'service_name', self.callback_function)

# Service Client
self.client = self.create_client(SrvType, 'service_name')
self.client.call_async(request)
```

### Action Client-Server

Used for long-running tasks with feedback:

```python
# Action Server
self._action_server = ActionServer(
    self,
    ActionType,
    'action_name',
    self.execute_callback)

# Action Client
self._action_client = ActionClient(self, ActionType, 'action_name')
self._send_goal_future = self._action_client.send_goal_async(goal_msg)
```

## Security Considerations

### Message Integrity

- Use checksums for hardware communication messages
- Validate message fields before processing
- Implement sequence numbers for multi-part messages

### Access Control

- Restrict access to critical services (thruster control, torpedo firing)
- Implement authorization for dangerous operations
- Use namespaces to isolate subsystems

### Fault Isolation

- Monitor communication timeouts
- Implement watchdog timers
- Design graceful degradation for communication failures

## Future Interface Improvements

### Proposed Enhancements

- **Quality of Service Policies**: Configure QoS for critical topics
- **Service Timeouts**: Add timeout parameters to long-running services
- **Heartbeat Mechanisms**: Implement heartbeat topics for component health monitoring
- **Interface Documentation Generation**: Automated generation of interface docs from message definitions
- **Diagnostic Interfaces**: Standardized diagnostic topics for all components