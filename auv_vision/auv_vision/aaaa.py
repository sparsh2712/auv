#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from pathlib import Path
import cv2
import depthai as dai
from auv_msgs.msg import AuvState, VisionSetpoints, ObjectDetected, BoundingBox, StereoVisionFrame, BBoxes
from auv_msgs.srv import VisionModel

class OakDDriver(Node):
    def __init__(self):
        super().__init__('depthai_tracker')
        self.image_pub = self.create_publisher(Image, '/front_camera/image_raw', 10)
        self.depth_frame_pub = self.create_publisher(StereoVisionFrame, '/front_camera/oakd_frame', 10)
        #self.task_toggle=self.create_service(VisionModel, '/vision/model', self.toggle_task)
        self.bbox_image_pub = self.create_publisher(Image, '/camera/bbox_image', 10)
        self.detector_pub = self.create_publisher(VisionSetpoints, '/vision/rel_position', 10)
        self.bbox_publish = self.create_publisher(BBoxes, '/vision/oakd_bbox', 10)

        self.task_toggle=self.create_service(VisionModel, '/vision/model', self.toggle_task)
                
        self.dvl_to_camera_vector = [0,0,0]
        self.cur_pose = AuvState()
        self.bridge = CvBridge()
        self.using_oakd = False
        self.task_name = 'Manit'

    def publish_bbox(self, bbox_list):
        bboxes = BBoxes()
        bboxes.bboxes = bbox_list
        self.bbox_publish.publish(bboxes)

    def toggle_task(self, req, res):
        self.using_oakd = req
        print('OakD status set to :',self.using_oakd)

    def publish_setpoint(self, objects_detectedData):
        v_vision = VisionSetpoints()
        for t in objects_detectedData:
            v_object = ObjectDetected()
            position = [t.spatialCoordinates.x, t.spatialCoordinates.y, t.spatialCoordinates.z]
            v_object.position.x = position[0]/10
            v_object.position.y = position[1]/10
            v_object.position.z = position[2]/10
            v_vision.objects.append(v_object)
        self.detector_pub.publish(v_vision)

    def toggle_task(self, req, res):
        self.task_name = req.model
        print('task set to :', self.task_name)

    def image_publisher(self, frame):
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(ros_image)

    def bbox_img_publisher(self, frame):
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.bbox_image_pub.publish(ros_image)

    def depth_frame_publisher(self, frame, depth_array):
        stereo_frame = StereoVisionFrame()
        stereo_frame.depth_frame = depth_array.flatten().tolist()
        stereo_frame.camera_frame = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.depth_frame_pub.publish(stereo_frame)


rclpy.init(args=None)
node = OakDDriver()

FPS = 10

labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

nnPath = str((Path(__file__).parent / Path('mobilenet-ssd_openvino_2021.4_5shave.blob')).resolve().absolute())

# Create pipeline
pipeline = dai.Pipeline()

# sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
objectTracker = pipeline.create(dai.node.ObjectTracker)
spatialDetectionNetwork = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutDepth = pipeline.create(dai.node.XLinkOut)
trackerOut = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("image")
trackerOut.setStreamName("objects_detected")

# camera properties
#camRgb.setPreviewSize(300,300)
camRgb.setPreviewSize(1280, 720)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
camRgb.setFps(FPS)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
monoLeft.setCamera("left")
monoLeft.setFps(FPS)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
monoRight.setCamera("right")
monoRight.setFps(FPS)

# depth stream
xoutDepth.setStreamName("depth")

# setting node configs
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())
stereo.setLeftRightCheck(True)
stereo.setSubpixel(True)

# spatial detection network params
spatialDetectionNetwork.setBlobPath(nnPath)
spatialDetectionNetwork.setConfidenceThreshold(0.5)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5) # bbox scaling factor matlab ratio of size of x and y from center, to be taken for depth calculation (average)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(5000)

    # object tracker params
objectTracker.setDetectionLabelsToTrack([15])  # track only person
objectTracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM) # possible tracking types: ZERO_TERM_COLOR_HISTOGRAM, ZERO_TERM_IMAGELESS, SHORT_TERM_IMAGELESS, SHORT_TERM_KCF
objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID) # take the smallest ID when new object is tracked, possible options: SMALLEST_ID, UNIQUE_ID

# Linking
stereo.depth.link(xoutDepth.input)
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)
camRgb.preview.link(spatialDetectionNetwork.input)
objectTracker.passthroughTrackerFrame.link(xoutRgb.input)
objectTracker.out.link(trackerOut.input)
spatialDetectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)
spatialDetectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
spatialDetectionNetwork.out.link(objectTracker.inputDetections)
stereo.depth.link(spatialDetectionNetwork.inputDepth)


# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    preview = device.getOutputQueue("image", 4, False)
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False) # output queue will be used to get the depth frames from the outputs defined above
    objects_detected = device.getOutputQueue("objects_detected", 4, False)
    
    color = (255, 255, 255)

    while(True):
        imgFrame = preview.get()
        inDepth = depthQueue.get() # Blocking call, will wait until a new data has arrived
        track = objects_detected.get()
        objects_detectedData = track.tracklets

        depthFrame = inDepth.getFrame() # depthFrame values are in millimeters
        frame = imgFrame.getCvFrame()

        node.depth_frame_publisher(frame, depthFrame)
        node.image_publisher(frame)
        if node.using_oakd:#node.models[node.task_name] == 'YOLO':
            objects_detected_msg = ''
            bbox = BoundingBox()
            node.publish_setpoint(objects_detectedData)
            bboxes = []
            for t in objects_detectedData:
                roi = t.roi.denormalize(frame.shape[1], frame.shape[0])
                x1 = int(roi.topLeft().x)
                y1 = int(roi.topLeft().y)
                x2 = int(roi.bottomRight().x)
                y2 = int(roi.bottomRight().y)

                bbox.class_id = 1#t.id
                bbox.class_name = node.task_name
                bbox.xmin = x1
                bbox.ymin = y1
                bbox.xmax = x2
                bbox.xmax = y2
                bbox.confidence = 0.99 if t.status.name == 'TRACKED' else 0.41
                bboxes.append(bbox)

                cv2.putText(frame, str(node.task_name), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"ID: {[t.id]}", (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, t.status.name, (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)

                cv2.putText(frame, f"X: {int(t.spatialCoordinates.x)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"Y: {int(t.spatialCoordinates.y)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"Z: {int(t.spatialCoordinates.z)} mm", (x1 + 10, y1 + 95), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

            cv2.putText(frame, "NN fps: {:.2f}".format(FPS), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
            
            node.publish_bbox(bboxes)
            node.bbox_img_publisher(frame)
        #cv2.imshow("tracker", frame)

        if cv2.waitKey(1) == ord('q'):
            break
