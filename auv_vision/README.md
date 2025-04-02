# AUV Vision Package

## Overview

The `auv_vision` package provides perception capabilities for the AUV, enabling it to detect and locate objects of interest underwater. It implements several computer vision approaches, from traditional CV methods to deep learning, and handles the challenges specific to underwater vision such as color distortion and variable lighting.

## Components

### Main Detector (`main_detector.py`)

The main detection system that orchestrates the vision pipeline:

- Handles camera input processing
- Routes detection requests to the appropriate algorithm
- Processes detection results into a standard format
- Manages 3D localization of detected objects

### Vision Algorithms

The package implements multiple detection approaches:

1. **Classical CV** (`bin_detection.py`, `buoy_detection.py`, `face_detection.py`):
   - Color-based segmentation with HSV filtering
   - Contour detection and analysis
   - Haar cascade detection (for testing/development)

2. **YOLOv5/v8** (`yolo_v5.py`):
   - Deep learning object detection
   - Pretrained models for competition elements
   - Custom models for specific tasks

3. **Water Removal** (`water_removal.py`):
   - Preprocessing to handle underwater color distortion
   - HSV-based filtering to isolate objects from water background
   - Enhances contrast for better detection

### Task Locator (`task_locator.py`)

Transforms 2D detections into 3D world coordinates:

- Uses relative positions and camera information
- Maps detections to known object dimensions
- Integrates with the AUV's localization system
- Maintains a map of task locations

### Post Processor (`post_processor.py`)

Filters and refines detection results:

- Time-series filtering through a sliding window approach
- Removes spurious detections
- Tracks detection confidence over time
- Improves stability of detection results

### Stereo Vision (`stereo_vision.py`)

Processes depth information from stereo cameras:

- Integrates with OAK-D depth camera
- Maps 2D detections to 3D using depth information
- Provides more accurate object distance estimates

## ROS2 Interface

### Published Topics

- `/vision/detection` (`auv_msgs/VisionSetpoints`): Detected objects and their 3D positions
- `/camera/bbox_image` (`sensor_msgs/Image`): Debug image with detection bounding boxes
- `/vision/oakd_bbox` (`auv_msgs/BBoxes`): Bounding boxes from OAK-D camera

### Subscribed Topics

- `/front_camera/image_raw` (`sensor_msgs/Image`): Front camera feed
- `/bottom_camera/image_raw` (`sensor_msgs/Image`): Bottom camera feed
- `/front_camera/oakd_frame` (`auv_msgs/StereoVisionFrame`): OAK-D stereo camera feed with depth
- `/localization/pose` (`auv_msgs/AuvState`): Current vehicle position for 3D mapping

### Services

- `/vision/model` (`auv_msgs/VisionModel`): Set current task/model for detection
- `/vision/toggle_camera` (`auv_msgs/SetDetector`): Switch between cameras
- `/vision/set_detector` (`auv_msgs/SetDetector`): Configure detector parameters

## Detection Tasks

The vision system is configured for detecting various competition elements:

1. **Buoys**: Colored spherical markers
2. **Bins**: Rectangular targets on the pool floor
3. **Torpedoes**: Small circular targets for torpedo shooting
4. **Gates**: Large rectangular structures to navigate through
5. **Octagon**: Octagonal structure for orientation

Each task has specific detection parameters and models configured in the system.

## Usage

### Prerequisites

- ROS2 environment set up
- `auv_msgs` package installed
- Camera drivers configured
- Python dependencies including OpenCV, NumPy, PyTorch, and Ultralytics

### Starting the Vision System

```bash
ros2 launch auv_vision auv_vision.launch.py
```

### Setting Detection Task

```bash
# Set the current detection task
ros2 service call /vision/model auv_msgs/srv/VisionModel "{model: 'buoy'}"
```

### Toggling Cameras

```bash
# Switch to front camera
ros2 service call /vision/toggle_camera auv_msgs/srv/SetDetector "{camera: 'front'}"

# Switch to bottom camera
ros2 service call /vision/toggle_camera auv_msgs/srv/SetDetector "{camera: 'bottom'}"
```

### Visualization

```bash
# View detection bounding boxes
ros2 run rqt_image_view rqt_image_view /camera/bbox_image
```

## Configuration

### Object Definitions

Objects are defined in `vision_constants.py` with properties like:

```python
{
    "object_id": 2,
    "object_name": "Buoy",
    "object_length": 15,
    "object_height": 15,
    "agreement_threshold": 20,
    "min_confidence": 0.4
}
```

### Task Definitions

Tasks are defined with their component objects and relative positions:

```python
{
    "task_name": "Buoy",
    "objects": [
        {
            "name": "Buoy",
            "rel_vector_from_task": {"x": 0, "y": 0, "z": 0},
            "priority": 1
        }
    ]
}
```

### CV Parameters

Color detection parameters are defined in `constants.py`:

```python
color_const = {
    'red': ([0,100,100],[200,255,255]),
    'blue': ([106,0,0],[130,255,255]),
    'buoy_red': ([70,100,100],[150,255,255]),
    'bin_red': ([150,0,0],[200,255,255])
}
```

## Algorithm Details

### Traditional CV Pipeline

1. **Water Removal**: Filter out water background
2. **Color Segmentation**: Isolate regions of interest using HSV thresholds
3. **Morphological Operations**: Clean up the binary mask
4. **Contour Detection**: Find object boundaries
5. **Geometric Filtering**: Remove false positives using shape criteria
6. **Bounding Box Generation**: Create bounding boxes around valid contours

### YOLOv8 Pipeline

1. **Preprocessing**: Normalize and resize input image
2. **Inference**: Run the YOLO model to detect objects
3. **Post-processing**: Filter detections based on confidence
4. **Class Mapping**: Map model classes to task objects
5. **Bounding Box Generation**: Create standardize