import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import depthai as dai
import cv2
import time

class DepthAITrackerNode(Node):
    def __init__(self):
        super().__init__('depthai_tracker')

        # Publishers
        self.image_pub = self.create_publisher(Image, 'tracker/image', 10)
        self.tracklets_pub = self.create_publisher(String, 'tracker/tracklets', 10)

        # CV bridge for ROS 2 Image messages
        self.bridge = CvBridge()

        # Initialize DepthAI pipeline
        self.pipeline = self.create_pipeline()
        self.device = dai.Device(self.pipeline)

        self.preview_queue = self.device.getOutputQueue("preview", 4, False)
        self.tracklets_queue = self.device.getOutputQueue("tracklets", 4, False)

        # FPS calculation
        self.start_time = time.monotonic()
        self.counter = 0
        self.fps = 0

        # Timer callback
        self.timer = self.create_timer(0.1, self.timer_callback)

    def create_pipeline(self):
        label_map = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
                     "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

        pipeline = dai.Pipeline()

        # Define DepthAI nodes
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)
        spatial_detection_network = pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
        object_tracker = pipeline.create(dai.node.ObjectTracker)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        tracker_out = pipeline.create(dai.node.XLinkOut)

        xout_rgb.setStreamName("preview")
        tracker_out.setStreamName("tracklets")

        # Camera properties
        cam_rgb.setPreviewSize(300, 300)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_800_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setCamera("left")
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setCamera("right")

        # Stereo depth configuration
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

        # Neural network configuration
        spatial_detection_network.setBlobPath(
            "depthai-python/examples/models/mobilenet-ssd_openvino_2021.4_5shave.blob")
        spatial_detection_network.setConfidenceThreshold(0.5)
        spatial_detection_network.setBoundingBoxScaleFactor(0.5)
        spatial_detection_network.setDepthLowerThreshold(100)
        spatial_detection_network.setDepthUpperThreshold(5000)

        # Object tracker configuration
        object_tracker.setDetectionLabelsToTrack([15])  # Track only 'person'
        object_tracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
        object_tracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)

        # Linking
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        cam_rgb.preview.link(spatial_detection_network.input)
        object_tracker.passthroughTrackerFrame.link(xout_rgb.input)
        object_tracker.out.link(tracker_out.input)
        spatial_detection_network.passthrough.link(object_tracker.inputDetectionFrame)
        spatial_detection_network.out.link(object_tracker.inputDetections)
        stereo.depth.link(spatial_detection_network.inputDepth)

        return pipeline

    def timer_callback(self):
        img_frame = self.preview_queue.get()
        track = self.tracklets_queue.get()

        self.counter += 1
        current_time = time.monotonic()
        if (current_time - self.start_time) > 1:
            self.fps = self.counter / (current_time - self.start_time)
            self.counter = 0
            self.start_time = current_time

        frame = img_frame.getCvFrame()
        tracklets_data = track.tracklets

        # Publish image
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.image_pub.publish(ros_image)

        # Publish tracklets
        tracklets_msg = ""
        for t in tracklets_data:
            roi = t.roi.denormalize(frame.shape[1], frame.shape[0])
            x1, y1 = int(roi.topLeft().x), int(roi.topLeft().y)
            x2, y2 = int(roi.bottomRight().x), int(roi.bottomRight().y)
            tracklet_info = (f"ID: {t.id}, Status: {t.status.name}, Coordinates: X={t.spatialCoordinates.x} mm, "
                             f"Y={t.spatialCoordinates.y} mm, Z={t.spatialCoordinates.z} mm\n")
            tracklets_msg += tracklet_info

        self.tracklets_pub.publish(String(data=tracklets_msg))

        # Optional: Draw tracklets on the frame for visualization
        for t in tracklets_data:
            roi = t.roi.denormalize(frame.shape[1], frame.shape[0])
            x1, y1 = int(roi.topLeft().x), int(roi.topLeft().y)
            x2, y2 = int(roi.bottomRight().x), int(roi.bottomRight().y)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)
            cv2.putText(frame, f"ID: {t.id}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        cv2.imshow("DepthAI Tracker", frame)
        if cv2.waitKey(1) == ord('q'):
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DepthAITrackerNode()
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

