#!/usr/bin/python3
import os
from typing import List
import cv2
import time
import rclpy
from rclpy.context import Context
from rclpy.node import Node
import signal
import threading
import subprocess

from cv_bridge import CvBridge
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image

# If there is some issue with killing the stream
# Kill 2 processes grep for gopro and v4l2

class CamMainGoPro(Node):
    def __init__(self):
        super().__init__('cam_main_go_pro')
        t1 = threading.Thread(target=self.cam_setup)
        t1.daemon = True
        t1.start()
        time.sleep(5)
        self.image = None
        self.br = CvBridge()
        self.cam_pub = self.create_publisher(Image, "/bottom_camera/image_raw", 1)
        self.timer = self.create_timer(0.1, self.run)
    
    def cam_setup(self):
        cam_setup_cmd="sudo gopro webcam --fov wide --resolution 1080"
        stream_setup_cmd="ffmpeg -nostdin -threads 2 -i 'udp://@0.0.0.0:8554?overrun_nonfatal=1&fifo_size=50000000' \
                -f:v mpegts -fflags nobuffer -vf format=yuv420p -f v4l2 /dev/video42"
        auto_start_cmd ="echo qwerty | sudo -S gopro webcam --fov wide --resolution 1080 -a"
        print("Camera setup done")
        os.system(auto_start_cmd)

    def publish(self):
        self.cam_pub.publish(self.br.cv2_to_imgmsg(self.image, encoding='rgb8'))
    
    def run(self):
        cap = cv2.VideoCapture("/dev/video42")
        frame_idx = 0
        print("Started Publishing")
        while(cap.isOpened()):
            ret, self.image = cap.read()
            self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
            frame_idx += 1
            self.publish()
            if cv2.waitKey(20) & 0xFF == ord('q'):
                break
    
def handle_iterrupt(signum, frame):
    print("interrupt handled")
    cam_stop_cmd="curl -s 172.20.146.51/gp/gpWebcam/STOP"
    os.system(cam_stop_cmd)
    exit()

def main(args=None):
    signal.signal(signal.SIGINT, handle_iterrupt)
    rclpy.init(args=args)
    cam_main_go_pro = CamMainGoPro()
    rclpy.spin(cam_main_go_pro)
    cam_main_go_pro.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()











