#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from pathlib import Path
import cv2
import depthai as dai
import numpy as np
from scipy.spatial.transform import Rotation
import time
import argparse
from auv_msgs.msg import AuvState, VisionSetpoints, ObjectDetected
from auv_msgs.srv import VisionModel

class DepthAITrackerNode(Node):
    def __init__(self):
        super().__init__('depthai_tracker')
        self.image_pub = self.create_publisher(Image, '/front_camera/image_raw', 10)
        self.tracklets_pub = self.create_publisher(String, 'tracker/tracklets', 10)
        self.create_subscription(AuvState, '/localization/pose', self.cb_pose, 10)
        #self.task_toggle=self.create_service(VisionModel, '/vision/model', self.toggle_task)
        self.bbox_image_pub = self.create_publisher(Image, '/camera/bbox_image', 10)
        self.detector_pub = self.create_publisher(VisionSetpoints, '/vision/rel_position', 10)

        self.task_toggle=self.create_service(VisionModel, '/vision/model', self.toggle_task)
       

        self.dvl_to_camera_vector = [0,0,0]
        self.cur_pose = AuvState()
        self.models = {'bin': 'CV', 'buoy': 'YOLO'}
        self.bridge = CvBridge()
        self.task_name = 'bouy'

    def toggle_task(self, req, res):
        self.task_name = req.model
        print('task set to :')

    def publish_setpoint(self, tracklet_data):
        v_vision = VisionSetpoints()
        for t in trackletsData:
            v_object = ObjectDetected()
            position = [t.spatialCoordinates.x, t.spatialCoordinates.y, t.spatialCoordinates.z]
            v_object.position.x = position[0]
            v_object.position.y = position[1]
            v_object.position.z = position[2]
            v_vision.objects.append(v_object)
        self.detector_pub.publish(v_vision)

    def toggle_task(self, req, res):
        self.task_name = req.model
        print('task set to :', self.task_name)

    def cb_pose(self, pose):
        self.cur_pose = pose

    def image_publisher(self, frame):
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(ros_image)

    def setp_publisher(self, tracklets_msg):
        print(tracklets_msg)
        self.tracklets_pub.publish(String(data=tracklets_msg))

    def bbox_publisher(self, frame):
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.bbox_image_pub.publish(ros_image)


rclpy.init(args=None)
node = DepthAITrackerNode()


labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

nnPath = str((Path(__file__).parent / Path('mobilenet-ssd_openvino_2021.4_5shave.blob')).resolve().absolute())
fullFrameTracking = False

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
spatialDetectionNetwork = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
objectTracker = pipeline.create(dai.node.ObjectTracker)

xoutRgb = pipeline.create(dai.node.XLinkOut)
trackerOut = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("preview")
trackerOut.setStreamName("tracklets")

# Properties
camRgb.setPreviewSize(300, 300)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

# setting node configs
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
# Align depth map to the perspective of RGB camera, on which inference is done
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())

spatialDetectionNetwork.setBlobPath(nnPath)
spatialDetectionNetwork.setConfidenceThreshold(0.5)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(5000)

objectTracker.setDetectionLabelsToTrack([15])  # track only person
# possible tracking types: ZERO_TERM_COLOR_HISTOGRAM, ZERO_TERM_IMAGELESS, SHORT_TERM_IMAGELESS, SHORT_TERM_KCF
objectTracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
# take the smallest ID when new object is tracked, possible options: SMALLEST_ID, UNIQUE_ID
objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

camRgb.preview.link(spatialDetectionNetwork.input)
objectTracker.passthroughTrackerFrame.link(xoutRgb.input)
objectTracker.out.link(trackerOut.input)

if fullFrameTracking:
    camRgb.setPreviewKeepAspectRatio(False)
    camRgb.video.link(objectTracker.inputTrackerFrame)
    objectTracker.inputTrackerFrame.setBlocking(False)
    # do not block the pipeline if it's too slow on full frame
    objectTracker.inputTrackerFrame.setQueueSize(2)
else:
    spatialDetectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)

spatialDetectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
spatialDetectionNetwork.out.link(objectTracker.inputDetections)
stereo.depth.link(spatialDetectionNetwork.inputDepth)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    preview = device.getOutputQueue("preview", 4, False)
    tracklets = device.getOutputQueue("tracklets", 4, False)

    startTime = time.monotonic()
    counter = 0
    fps = 0
    color = (255, 255, 255)

    while(True):
        imgFrame = preview.get()
        track = tracklets.get()

        counter += 1
        current_time = time.monotonic()
        if (current_time - startTime) > 1:
            fps = counter / (current_time - startTime)
            counter = 0
            startTime = current_time

        frame = imgFrame.getCvFrame()
        trackletsData = track.tracklets
        if True:#node.models[node.task_name] == 'YOLO':
            tracklets_msg = ''
            for t in trackletsData:
                #node.publish_setpoint()
                roi = t.roi.denormalize(frame.shape[1], frame.shape[0])
                x1 = int(roi.topLeft().x)
                y1 = int(roi.topLeft().y)
                x2 = int(roi.bottomRight().x)
                y2 = int(roi.bottomRight().y)

                cv2.putText(frame, str(node.task_name), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"ID: {[t.id]}", (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, t.status.name, (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)

                cv2.putText(frame, f"X: {int(t.spatialCoordinates.x)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"Y: {int(t.spatialCoordinates.y)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"Z: {int(t.spatialCoordinates.z)} mm", (x1 + 10, y1 + 95), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

            node.publish_setpoint(trackletsData)
            cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
        
            node.bbox_publisher(frame)
            cv2.imshow("tracker", frame)

        if cv2.waitKey(1) == ord('q'):
            break
