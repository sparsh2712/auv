#!/usr/bin/env python3

import os
import cv2
import time
import numpy as np
import rclpy

import rospkg
from sensor_msgs.msg import Image
from auv_msgs.msg import StereoVisionFrame
from cv_bridge import CvBridge

import sys

sys.path[0] = "/usr/lib/python3"

if __name__ == '__main__':
    args = None
    rclpy.init(args=args)
    node = rclpy.create_node('front_camera')
    print("!!! Test front camera launched!!!")

    image_pub = node.create_publisher(StereoVisionFrame, '/front_camera/oakd_frame', 1)

    # Clear the test_output folder
    output_folder = "test_output"

    bridge = CvBridge()
    rospack = rospkg.RosPack()

    time.sleep(1)

    # Read all images from the test_images folder

    input_folder = "frame_depth_folder"
    file_list = os.listdir(input_folder)
    for i in range(177):
        frame = StereoVisionFrame()
        file_name = 'image_'+str(i)+'.png'
        file_path = os.path.join(input_folder, file_name)
        cur_img = cv2.imread(file_path)
        array_name = 'array_'+str(i)+'.npy'
        file_path = os.path.join(input_folder,array_name)
        array = np.load(file_path)
        if cur_img is not None:
            frame.camera_frame = bridge.cv2_to_imgmsg(cur_img, encoding="bgr8")
            frame.depth_frame = array.flatten().tolist()
            image_pub.publish(frame)
            print("Published image:", file_name)
        else:
            print("Failed to read image:", file_name)
        
        # Add a delay between publishing images
        time.sleep(1)
