import os
import cv2
import rclpy
from rclpy.node import Node
import signal
import numpy as np
from pypylon import pylon

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class CamMainBasler(Node):
    def __init__(self):
        super().__init__('cam_main_basler')
        self.image = None
        self.br = CvBridge()
        self.cam_pub=self.create_publisher(Image, "/front_camera/image_raw", 1)
        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
        self.camera.Open()

        self.camera.Width.SetValue(self.camera.Width.GetMax())
        self.camera.Height.SetValue(self.camera.Height.GetMax())

        self.counter = 0

        self.camera.StartGrabbing()
        self.timer = self.create_timer(0.1, self.run)
    
    def publish(self):
        self.cam_pub.publish(self.br.cv2_to_imgmsg(self.image, encoding='rgb8'))
    
    def run(self):
        frame_idx = 0
        print("Started Publishing")
        while self.camera.IsGrabbing():
            self.counter += 1
            grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if grabResult.GrabSucceeded():
                self.image = grabResult.Array
                self.image = cv2.cvtColor(self.image, cv2.COLOR_BayerRG2RGB)
                if self.counter % 35 == 0:
                    cv2.imwrite("/home/auv/catkin_ws/src/matsya/auv_drivers/scripts/cam/test_images_basler/img_"+str(self.counter)+".jpg", self.image)
                # self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
                #print(type(self.image))
                frame_idx += 1
                # print("Frame num: " + str(frame_idx))
                self.publish()
            grabResult.Release()
def handle_iterrupt(signum, frame):
    print("interrupt handled")
    exit()

def main(args=None):
    signal.signal(signal.SIGINT, handle_iterrupt)
    rclpy.init(args=args)
    cam_main_basler = CamMainBasler()
    rclpy.spin(cam_main_basler)
    cam_main_basler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





