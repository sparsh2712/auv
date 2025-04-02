#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import numpy as np
from datetime import datetime
import cv2
import os
import rospkg
from threading import Lock
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import depthai as dai
import time
import argparse
from sensor_msgs.msg import Image
from auv_msgs.msg import AuvState, BoundingBox, VisionSetpoints, ObjectDetected, TaskDetected
from auv_msgs.srv import SetDetector, VisionModel, SetTorpedo

from .post_processor import PostProcessor
from .main_detector import MainDetector
from .task_locator import TaskLocator
from .vision_constants import get_vision_params, get_task_details 

package_path = get_package_share_directory('auv_vision')
PATH = package_path + '/src'

workspace_path = '/home/ubuntu/ros2_ws'
package_name = 'auv_vision'
src_path = os.path.join(workspace_path, 'src', 'Robosub_ROS2_2024', package_name, package_name)

class Object:
    def __init__(self, object_id, object_name, object_length, object_height,
        agreement_threshold, min_confidence, rel_midpoint=[0, 0, 0], glob_midpoint=[0, 0, 0]):
        self.object_id=object_id
        self.object_name=object_name
        self.object_length=object_length
        self.object_height=object_height
        self.agreement_threshold=agreement_threshold
        self.min_confidence=min_confidence
        self.rel_midpoint=rel_midpoint
        self.glob_midpoint=glob_midpoint
        self.object_detected=False
    
    def pprint(self):
        print("Object ID:", self.object_id)
        print("Object Name:", self.object_name)
        print("Object Length:", self.object_length)
        print("Object Height:", self.object_height)
        print("Min Confidence:", self.min_confidence)
        print("Relative Midpoint:", self.rel_midpoint)
        print("Global Midpoint:", self.glob_midpoint)
        print("Object Detected:", self.object_detected)

class Stereo_Vision(Node):
    lock=Lock()
    flag=False

    def __init__(self, front_camera_config, bottom_camera_config,
        dvl_to_front_cam_vector, dvl_to_bottom_cam_vector, objects, task_details):
        super().__init__('vision_node')
        # initialise CvBridge, Yolov5, main detector
        # initialise camera config, cam vector
        # list objects
        #write subscribers and publishers and services


        self.br = CvBridge()
        print("Hi")
        print("Ho")
        self.camera = "front" # could change it to no_cam, if you don't want to start vision until necessary

        self.front_camera_config = front_camera_config
        self.bottom_camera_config = bottom_camera_config
        self.dvl_to_front_cam_vector = [dvl_to_front_cam_vector["x"], dvl_to_front_cam_vector["y"], dvl_to_front_cam_vector["z"]]
        self.dvl_to_bottom_cam_vector = [dvl_to_bottom_cam_vector["x"], dvl_to_bottom_cam_vector["y"], dvl_to_bottom_cam_vector["z"]]
        self.task_details = task_details

        self.counter = 0
        self.fr_frame = 0
        self.bt_frame = 0

        #self.target_img_path = PATH + "/reference_image3.jpg"
        #self.target_img = cv2.imread(self.target_img_path)
        #self.target_img = cv2.cvtColor(self.target_img, cv2.COLOR_RGB2LAB)

        self.cur_pose = AuvState()
        self.post_processors = {}

        object_list = []
        if objects is not None:
            for object in objects:
                cur_object = Object(object["object_id"], object["object_name"],
                    object["object_length"], object["object_height"], object["agreement_threshold"], object["min_confidence"])
                object_list.append(cur_object)
                self.post_processors[object["object_name"]] = PostProcessor(object["object_name"], object["agreement_threshold"])
            
        self.objects = object_list
        
        self.main_detector = MainDetector(self.task_details, camera="front", config=self.front_camera_config, objects = self.objects)
        self.task_locator = TaskLocator(task_details, self.objects, self.dvl_to_front_cam_vector, self.camera)
        
        self.create_subscription(Image, '/front_camera/image_raw', self.cb_front_cam_img, 10)
        self.create_subscription(Image, '/bottom_camera/image_raw', self.cb_bottom_cam_img, 10)
        self.create_subscription(AuvState, '/localization/pose', self.cb_pose, 10)

        self.detector_toggle_srv = self.create_service(SetDetector, '/vision/toggle_camera', self.toggle_detection)
        self.task_toggle=self.create_service(VisionModel, '/vision/model', self.toggle_task)
         
        self.detector_pub = self.create_publisher(VisionSetpoints, '/vision/detection', 10)
        self.bbox_image_pub = self.create_publisher(Image, '/camera/bbox_image', 10)
        self.task_name="buoy"

        #stero_vision params
        self.labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]
        self.fullFrameTracking = True
        self.showDetections = False 
        self.nnPath = str((Path(__file__).parent / Path('depthai-python/examples/models/mobilenet-ssd_openvino_2021.4_5shave.blob')).resolve().absolute())
        # Create pipeline
        self.pipeline = dai.Pipeline()

        # Define sources and outputs
        camRgb = self.pipeline.create(dai.node.ColorCamera)
        spatialDetectionNetwork = self.pipeline.create(dai.node.MobileNetSpatialDetectionNetwork)
        monoLeft = self.pipeline.create(dai.node.MonoCamera)
        monoRight = self.pipeline.create(dai.node.MonoCamera)
        stereo = self.pipeline.create(dai.node.StereoDepth)
        objectTracker = self.pipeline.create(dai.node.ObjectTracker)

        xoutRgb = self.pipeline.create(dai.node.XLinkOut)
        trackerOut = self.pipeline.create(dai.node.XLinkOut)

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

        spatialDetectionNetwork.setBlobPath(self.nnPath)
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

        if self.fullFrameTracking:
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

        print("Vision Ready!!")

    def start_stereo_cam(self):
        with dai.Device(self.pipeline) as device:
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
                if self.showDetections:  # Only add detections if showDetections is True
                    for t in trackletsData:
                        roi = t.roi.denormalize(frame.shape[1], frame.shape[0])
                        x1 = int(roi.topLeft().x)
                        y1 = int(roi.topLeft().y)
                        x2 = int(roi.bottomRight().x)
                        y2 = int(roi.bottomRight().y)

                        try:
                            label = self.labelMap[t.label]
                        except:
                            label = t.label

                        cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                        cv2.putText(frame, f"ID: {[t.id]}", (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                        cv2.putText(frame, t.status.name, (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)

                        cv2.putText(frame, f"X: {int(t.spatialCoordinates.x)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                        cv2.putText(frame, f"Y: {int(t.spatialCoordinates.y)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                        cv2.putText(frame, f"Z: {int(t.spatialCoordinates.z)} mm", (x1 + 10, y1 + 95), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

                cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)

                cv2.imshow("tracker", frame)

    def toggle_task(self, req, res):
        self.task_name = req.model
        print('task set to :')

    def cb_pose(self, msg):
        self.cur_pose.position=msg.position
        print('Position updated to', msg.position)

    def toggle_detection(self, req, res):
        self.camera = req.camera
        print('camera requested :', req.camera)
        if req.camera == "front":
            self.detector = MainDetector(self.task_details, camera="front",
                config=self.front_camera_config)
            self.task_locator = TaskLocator(self.task_details, self.objects, self.dvl_to_front_cam_vector, self.camera)
        elif req.camera == "bottom":
            self.detector = MainDetector(self.task_details, camera="bottom",
                config=self.bottom_camera_config)
            self.task_locator = TaskLocator(self.task_details, self.objects, self.dvl_to_bottom_cam_vector, self.camera)
        return res

    def cb_front_cam_img(self, img_msg):
        if self.camera == "front" and Stereo_Vision.flag == False:
            self.fr_frame += 1
            self.run_detection(img_msg, self.main_detector, self.task_name)
    
    def cb_bottom_cam_img(self, img_msg):
        if self.camera == "bottom" and Stereo_Vision.flag == False:
            self.fr_frame += 1 #self.bt_frame += 1
            self.main_detector.run_detection(img_msg, self.task_name)

    def run_detection(self, img_msg, detector, task_name):
        Stereo_Vision.lock.acquire()
        Stereo_Vision.flag=True
        print("Detections being done")
        
        final_img, bboxes, rel_pose_list = self.main_detector.run_detection(img_msg, self.task_name)
        self.bbox_image_pub.publish(self.br.cv2_to_imgmsg(final_img, encoding='bgr8'))
        print('done with detection')

        vision_setpoints_detected = self.task_locator.locate_tasks(bboxes, rel_pose_list, self.cur_pose)
        print(vision_setpoints_detected)
        vision_setpoints = VisionSetpoints()
        for cur_obj in vision_setpoints_detected.objects:
            obj_plot = {}
            obj_plot['x'] = cur_obj.position.x
            obj_plot['y'] = cur_obj.position.y
            obj_plot['z'] = cur_obj.position.z
            obj_plot['frame_num'] = self.fr_frame

            predicted_pose = self.post_processors[cur_obj.name].sliding_window_detect(obj_plot)

            if len(predicted_pose) != 0:
                    cur_obj.position.x = predicted_pose[0]
                    cur_obj.position.y = predicted_pose[1]
                    cur_obj.position.z = predicted_pose[2]
                
                    vision_setpoints.objects.append(cur_obj)
        self.detector_pub.publish(vision_setpoints)
        Stereo_Vision.lock.release()
        Stereo_Vision.flag = False

def main(args=None):
    rclpy.init(args=args)
    (front_camera_config, bottom_camera_config, dvl_to_front_cam_vector,
     dvl_to_bottom_cam_vector, objects) = get_vision_params()
    task_details = get_task_details()
    
    vision = Stereo_Vision(front_camera_config, bottom_camera_config,
        dvl_to_front_cam_vector, dvl_to_bottom_cam_vector, objects, task_details)
    try:
        rclpy.spin(vision)
    
    finally:
        vision.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
